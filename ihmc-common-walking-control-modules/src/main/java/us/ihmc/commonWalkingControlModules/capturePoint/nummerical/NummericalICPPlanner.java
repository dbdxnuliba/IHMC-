package us.ihmc.commonWalkingControlModules.capturePoint.nummerical;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ObjDoubleConsumer;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.TDoubleList;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class NummericalICPPlanner
{
   /** Weight for maintaining initial ICP continuity */
   private static final double ICP_WEIGHT = 1.0;
   /** Weight for maintaining CoP continuity */
   private static final double COP_WEIGHT = 0.0001;

   // Constants:
   private final int adjustmentSteps;
   private final double timestep;
   private final double omega;
   private final double initialIcpAdjustmentFactor;

   // Sampled trajectories:
   private final List<Point2DBasics> icps = new ArrayList<>();
   private final List<Point2DBasics> cops = new ArrayList<>();
   private final List<Vector2DBasics> angularMomentums = new ArrayList<>();

   // Helper trajectories for interpolating:
   private final SimpleTrajectory icpTrajectory;
   private final SimpleTrajectory copTrajectory;

   // Matrices for QP solver:
   private final DenseMatrix64F Q;
   private final DenseMatrix64F x;
   private final DenseMatrix64F f;
   private final QuadProgSolver solver = new QuadProgSolver();

   // Objective matrices for initial ICP:
   // min (AIcp * x - bIcp)' * WIcp * (AIcp * x - bIcp)
   private final DenseMatrix64F WIcp;
   private final DenseMatrix64F AIcp;
   private final DenseMatrix64F bIcp;
   // Pre-computed matrices:
   private final DenseMatrix64F AtWicp;
   // Initial ICP:
   private final Point2D initialIcp = new Point2D();

   // Finite difference matrix for penalizing CoP movement:
   // min (FD * x)' * COP_WEIGHT * (FD * x)
   private final DenseMatrix64F FD;
   private final DenseMatrix64F FDtFD;

   // Constraint matrices for initial and final CoP location:
   private final DenseMatrix64F Aeq;
   private final DenseMatrix64F beq;

   // Constraint matrices for support polygons:
   private final DenseMatrix64F bin = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Ain = new DenseMatrix64F(0, 0);
   // Temporary helper matrices for assembling the inequalities:
   private final DenseMatrix64F subAin = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F subbin = new DenseMatrix64F(0, 0);

   public NummericalICPPlanner(double timestep, double previewTime, double omega)
   {
      this(timestep, previewTime, omega, 0.0);
   }

   public NummericalICPPlanner(double timestep, double timeHorizon, double omega, double adjustmentTimeHorizon)
   {
      this.timestep = timestep;
      this.omega = omega;

      int timeSteps = (int) (timeHorizon / timestep) + 1;
      if (timeSteps <= 0)
         throw new RuntimeException("Invalid parameters.");

      for (int i = 0; i < timeSteps; i++)
         icps.add(new Point2D());
      for (int i = 0; i < timeSteps - 1; i++)
      {
         cops.add(new Point2D());
         angularMomentums.add(new Vector2D());
      }

      icpTrajectory = new SimpleTrajectory(icps, timestep * (icps.size() - 1));
      copTrajectory = new SimpleTrajectory(cops, timestep * (cops.size() - 1));

      // Limit to be between 2 (effectively no adjustment) and the total amount of steps
      adjustmentSteps = Math.max(Math.min((int) (adjustmentTimeHorizon / timestep), timeSteps - 1), 2);
      double alpha = 1.0 + timestep * omega;
      initialIcpAdjustmentFactor = Math.pow(alpha, adjustmentSteps - 1);

      // Initialize solver matrices
      Q = new DenseMatrix64F(2 * adjustmentSteps, 2 * adjustmentSteps);
      x = new DenseMatrix64F(2 * adjustmentSteps, 1);
      f = new DenseMatrix64F(2 * adjustmentSteps, 1);

      // Initialize matrices for the solving of the initial cop trajectory
      WIcp = new DenseMatrix64F(2, 2);
      AIcp = new DenseMatrix64F(2, 2 * adjustmentSteps);
      bIcp = new DenseMatrix64F(2, 1);
      AtWicp = new DenseMatrix64F(2 * adjustmentSteps, 2);
      for (int i = 0; i < adjustmentSteps; i++)
      {
         AIcp.set(0, 2 * i, -timestep * omega * Math.pow(alpha, adjustmentSteps - i - 1));
         AIcp.set(1, 2 * i + 1, -timestep * omega * Math.pow(alpha, adjustmentSteps - i - 1));
      }
      CommonOps.setIdentity(WIcp);
      CommonOps.scale(ICP_WEIGHT, WIcp);
      CommonOps.multTransA(AIcp, WIcp, AtWicp);

      // Initialize matrices for CoP equality constraints at start and end
      Aeq = new DenseMatrix64F(4, 2 * adjustmentSteps);
      beq = new DenseMatrix64F(4, 1);
      Aeq.set(0, 0, 1.0);
      Aeq.set(1, 1, 1.0);
      Aeq.set(2, 2 * adjustmentSteps - 2, 1.0);
      Aeq.set(3, 2 * adjustmentSteps - 1, 1.0);

      // Initialize finite difference matrices for minimizing CoP velocity
      FD = new DenseMatrix64F(2 * adjustmentSteps - 2, 2 * adjustmentSteps);
      FDtFD = new DenseMatrix64F(2 * adjustmentSteps, 2 * adjustmentSteps);
      for (int i = 0; i < adjustmentSteps - 1; i++)
      {
         FD.set(2 * i, 2 * i, -1.0 / timestep);
         FD.set(2 * i, 2 * i + 2, 1.0 / timestep);
         FD.set(2 * i + 1, 2 * i + 1, -1.0 / timestep);
         FD.set(2 * i + 1, 2 * i + 3, 1.0 / timestep);
      }
      CommonOps.multTransA(FD, FD, FDtFD);
      CommonOps.scale(COP_WEIGHT, FDtFD);

      // Initialize the quadratic cost term since it will remain constant
      Q.set(FDtFD);
      CommonOps.multAdd(AtWicp, AIcp, Q);
   }

   public void setCopTrajectory(ObjDoubleConsumer<Point2DBasics> copTrajectory)
   {
      // Re-sample the trajectory at the sample rate of this planner:
      for (int i = 0; i < cops.size(); i++)
      {
         double time = timestep * i;
         copTrajectory.accept(cops.get(i), time);
      }
   }

   public void setAngularMomentumTrajectory(ObjDoubleConsumer<Vector2DBasics> angularMomentumTrajectory)
   {
      // Re-sample the trajectory at the sample rate of this planner:
      for (int i = 0; i < angularMomentums.size(); i++)
      {
         double time = timestep * i;
         angularMomentumTrajectory.accept(angularMomentums.get(i), time);
      }
   }

   public void setFinalIcp(Point2DReadOnly finalIcp)
   {
      icps.get(icps.size() - 1).set(finalIcp);
   }

   public void setInitialIcp(Point2DReadOnly initialIcp)
   {
      this.initialIcp.set(initialIcp);
   }

   public void setCopConstraints(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportDurations)
   {
      bin.reshape(0, 1);
      Ain.reshape(0, 2 * adjustmentSteps);
      int index = 0;
      double supportEnd = supportDurations.get(index);
      for (int step = 0; step < adjustmentSteps; step++)
      {
         double time = timestep * step;
         while (time > supportEnd)
         {
            index++;
            supportEnd += supportDurations.get(index);
         }
         ConvexPolygon2DReadOnly supportPolygon = supportPolygons.get(index);
         PolygonWiggler.convertToInequalityConstraints(supportPolygon, subAin, subbin, 0.0);

         int currentConstraints = bin.getNumRows();
         int newConstraints = subbin.getNumRows();
         bin.reshape(currentConstraints + newConstraints, 1, true);
         Ain.reshape(currentConstraints + newConstraints, Ain.getNumCols(), true);
         CommonOps.insert(subAin, Ain, currentConstraints, 2 * step);
         CommonOps.insert(subbin, bin, currentConstraints, 0);
      }
   }

   public void compute()
   {
      reverseTimeIntegrate(icps.size() - 2, adjustmentSteps - 1);
      solveInitialCopTrajectory();
      reverseTimeIntegrate(adjustmentSteps - 1, 0);
   }

   private void solveInitialCopTrajectory()
   {
      // In this case there is nothing to adjust since the initial and final CoP location are constrained
      if (adjustmentSteps <= 2)
         return;

      // Compute angular momentum adjustment term
      double amX = 0.0;
      double amY = 0.0;
      for (int i = 0; i < adjustmentSteps; i++)
      {
         amX += AIcp.get(0, i) * angularMomentums.get(i).getX();
         amY += AIcp.get(1, i) * angularMomentums.get(i).getY();
      }

      // Setup initial ICP continuity objective
      bIcp.set(0, 0, -icps.get(adjustmentSteps - 1).getX() + initialIcp.getX() * initialIcpAdjustmentFactor + amY);
      bIcp.set(1, 0, -icps.get(adjustmentSteps - 1).getY() + initialIcp.getY() * initialIcpAdjustmentFactor - amX);
      NativeCommonOps.mult(AtWicp, bIcp, f);

      // Setup equality constraint for initial and final CoP
      beq.set(0, cops.get(0).getX());
      beq.set(1, cops.get(0).getY());
      beq.set(2, cops.get(adjustmentSteps - 1).getX());
      beq.set(3, cops.get(adjustmentSteps - 1).getY());

      try
      {
         solver.solve(Q, f, Aeq, beq, Ain, bin, x, true);
      }
      catch (NoConvergenceException e)
      {
         LogTools.warn("ICP trajectory computation failed, using reverse time integration only.");
         return;
      }

      for (int i = 0; i < adjustmentSteps; i++)
      {
         cops.get(i).setX(x.get(2 * i));
         cops.get(i).setY(x.get(2 * i + 1));
      }
   }

   private void reverseTimeIntegrate(int start, int end)
   {
      // Use numerical backwards integration to find the ICP trajectory:
      for (int i = start; i >= end; i--)
      {
         Point2DReadOnly nextIcp = icps.get(i + 1);
         Point2DBasics cop = cops.get(i);
         Point2DBasics icp = icps.get(i);
         Vector2DBasics angularMomentum = angularMomentums.get(i);

         icp.setX(nextIcp.getX() - omega * timestep * (nextIcp.getX() - cop.getX() - angularMomentum.getY()));
         icp.setY(nextIcp.getY() - omega * timestep * (nextIcp.getY() - cop.getY() + angularMomentum.getX()));
      }
   }

   public void getIcp(double time, Point2DBasics icpToPack)
   {
      icpTrajectory.accept(icpToPack, time);
   }

   public void getCop(double time, Point2DBasics copToPack)
   {
      copTrajectory.accept(copToPack, time);
   }
}
