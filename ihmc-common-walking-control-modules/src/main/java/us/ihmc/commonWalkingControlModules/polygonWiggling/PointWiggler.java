package us.ihmc.commonWalkingControlModules.polygonWiggling;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;

public class PointWiggler
{
   private static final boolean DEBUG = true;
   private static final boolean coldStart = true;

   /**
    * This method will find a translation that moves a point into a given convex region.
    * It holds the constraints rigidly. Therefore, it might not find a solution.
    */
   public static RigidBodyTransform findWiggleTransform(Point2DReadOnly pointToWiggle, ConvexPolygon2DReadOnly planeToWiggleInto, WiggleParameters parameters,
                                                        int[] startingVerticesToIgnore)
   {
      // This creates inequality constraints for points to lie inside the desired polygon.
      DenseMatrix64F A_inequality = new DenseMatrix64F(0);
      DenseMatrix64F b_inequality = new DenseMatrix64F(0);
      PolygonWiggler.convertToInequalityConstraints(planeToWiggleInto, A_inequality, b_inequality, parameters.deltaInside, startingVerticesToIgnore);

      int constraintsPerPoint = A_inequality.getNumRows();

      DenseMatrix64F p = new DenseMatrix64F(2, 1);
      p.set(0, pointToWiggle.getX());
      p.set(1, pointToWiggle.getY());

      DenseMatrix64F b_new = new DenseMatrix64F(constraintsPerPoint, 1);

      CommonOps.mult(A_inequality, p, b_new);
      System.out.println(b_new);

      CommonOps.subtract(b_inequality, b_new, b_inequality);
      System.out.println(b_inequality);

      System.out.println(A_inequality);
      System.out.println(b_inequality);

      int boundConstraints = 4;
      DenseMatrix64F A_full = new DenseMatrix64F(constraintsPerPoint + boundConstraints, 2);
      DenseMatrix64F b_full = new DenseMatrix64F(constraintsPerPoint + boundConstraints, 1);

      CommonOps.insert(A_inequality, A_full, 0, 0);
      CommonOps.insert(b_inequality, b_full, 0, 0);

      // add limits on allowed rotation and translation
      A_full.set(constraintsPerPoint, 0, 1.0);
      b_full.set(constraintsPerPoint, parameters.maxX);
      A_full.set(constraintsPerPoint + 1, 0, -1.0);
      b_full.set(constraintsPerPoint + 1, -parameters.minX);
      A_full.set(constraintsPerPoint + 2, 1, 1.0);
      b_full.set(constraintsPerPoint + 2, parameters.maxY);
      A_full.set(constraintsPerPoint + 3, 1, -1.0);
      b_full.set(constraintsPerPoint + 3, -parameters.minY);

      System.out.println("A_full:");
      System.out.println(A_full);

      System.out.println("b_full:");
      System.out.println(b_full);

      //      // Convert the inequality constraint for being inside the polygon to an objective.
      DenseMatrix64F costMatrix = CommonOps.identity(2);
      DenseMatrix64F costVector = new DenseMatrix64F(2, 1);

      QuadProgSolver solver = new QuadProgSolver();
      DenseMatrix64F result = new DenseMatrix64F(2, 1);
      DenseMatrix64F Aeq = new DenseMatrix64F(0, 2);
      DenseMatrix64F beq = new DenseMatrix64F(0, 1);
      try
      {
         int iterations = solver.solve(costMatrix, costVector, Aeq, beq, A_full, b_full, result, coldStart);
         if (DEBUG)
         {
            LogTools.info("Iterations: " + iterations);
            LogTools.info("Result: " + result);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }

      if (Double.isInfinite(solver.getCost()))
      {
         LogTools.info("Could not wiggle!");
         return null;
      }

      // assemble the transform
      Vector3D translation = new Vector3D(result.get(0), result.get(1), 0.0);
      RigidBodyTransform fullTransform = new RigidBodyTransform();
      fullTransform.setTranslation(translation);

      return fullTransform;
   }
}
