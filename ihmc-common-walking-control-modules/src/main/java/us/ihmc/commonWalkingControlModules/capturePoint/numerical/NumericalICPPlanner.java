package us.ihmc.commonWalkingControlModules.capturePoint.numerical;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.ObjDoubleConsumer;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.TDoubleList;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class NumericalICPPlanner
{
   /** Weight for maintaining initial ICP continuity */
   private static final double ICP_WEIGHT = 1.0;
   /** Weight for maintaining CoP continuity */
   private static final double COP_WEIGHT = 0.001;
   /** Whether the continuity at the initial CoP should be an equality constraint */
   private static final boolean INITIAL_COP_CONSTRAINT = false;
   /** Whether the continuity at the initial ICP should be an equality constraint */
   private static final boolean INITIAL_ICP_CONSTRAINT = false;
   /** The amount by which to shrink any CoP constraint polygons */
   private static final double COP_SAFETY_DISTANCE = 0.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private double omega;

   // Constants:
   private final int adjustmentSteps;
   private final double timestep;

   // Sampled trajectories:
   private final List<Point2DBasics> icps = new ArrayList<>();
   private final List<Point2DBasics> cops = new ArrayList<>();
   private final List<Point2DBasics> originalCops = new ArrayList<>();
   private final List<ConvexPolygon2DBasics> constraintPolygons = new ArrayList<>();
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
   private final DenseMatrix64F bin;
   private final DenseMatrix64F Ain;
   // Temporary helper matrices for assembling the inequalities:
   private final DenseMatrix64F subAin = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F subbin = new DenseMatrix64F(0, 0);

   public NumericalICPPlanner(double timestep, double previewTime)
   {
      this(timestep, previewTime, 0.0, new YoVariableRegistry("Parent"), null);
   }

   public NumericalICPPlanner(double timestep, double timeHorizon, double adjustmentTimeHorizon)
   {
      this(timestep, timeHorizon, adjustmentTimeHorizon, new YoVariableRegistry("Parent"), null);
   }

   public NumericalICPPlanner(double timestep, double previewTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      this(timestep, previewTime, 0.0, parentRegistry, graphicsRegistry);
   }

   public NumericalICPPlanner(double timestep, double timeHorizon, double adjustmentTimeHorizon, YoVariableRegistry parentRegistry,
                              YoGraphicsListRegistry graphicsRegistry)
   {
      this.timestep = timestep;

      int timeSteps = (int) (timeHorizon / timestep) + 1;
      if (timeSteps <= 0)
         throw new RuntimeException("Invalid parameters.");

      for (int i = 0; i < timeSteps; i++)
      {
         icps.add(new YoFramePoint2D("Icp" + i, ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < timeSteps - 1; i++)
      {
         cops.add(new YoFramePoint2D("Cop" + i, ReferenceFrame.getWorldFrame(), registry));
         angularMomentums.add(new Vector2D());
      }

      icpTrajectory = new SimpleTrajectory(icps, timestep * (icps.size() - 1));
      copTrajectory = new SimpleTrajectory(cops, timestep * (cops.size() - 1));

      // Limit to be between 1 (effectively no adjustment) and the total amount of steps
      adjustmentSteps = Math.max(Math.min((int) (adjustmentTimeHorizon / timestep), timeSteps - 1), 1);
      for (int i = 0; i < adjustmentSteps; i++)
      {
         originalCops.add(new YoFramePoint2D("OriginalCops" + i, ReferenceFrame.getWorldFrame(), registry));
         constraintPolygons.add(new YoFrameConvexPolygon2D("ConstraintPolygon" + i, ReferenceFrame.getWorldFrame(), 10, registry));
      }

      // Initialize solver matrices
      Q = new DenseMatrix64F(2 * adjustmentSteps, 2 * adjustmentSteps);
      x = new DenseMatrix64F(2 * adjustmentSteps, 1);
      f = new DenseMatrix64F(2 * adjustmentSteps, 1);
      bin = new DenseMatrix64F(0, 1);
      Ain = new DenseMatrix64F(0, 2 * adjustmentSteps);

      // Initialize matrices for the solving of the initial cop trajectory
      WIcp = new DenseMatrix64F(2, 2);
      AIcp = new DenseMatrix64F(2, 2 * adjustmentSteps);
      bIcp = new DenseMatrix64F(2, 1);
      AtWicp = new DenseMatrix64F(2 * adjustmentSteps, 2);
      CommonOps.setIdentity(WIcp);
      CommonOps.scale(ICP_WEIGHT, WIcp);

      // Initialize matrices for CoP equality constraints at start and end
      Aeq = new DenseMatrix64F(2, 2 * adjustmentSteps);
      beq = new DenseMatrix64F(2, 1);
      Aeq.set(0, 2 * adjustmentSteps - 2, 1.0);
      Aeq.set(1, 2 * adjustmentSteps - 1, 1.0);
      if (INITIAL_COP_CONSTRAINT)
      {
         Aeq.reshape(Aeq.getNumRows() + 2, Aeq.getNumCols(), true);
         beq.reshape(beq.getNumRows() + 2, beq.getNumCols(), true);
         Aeq.set(2, 0, 1.0);
         Aeq.set(3, 1, 1.0);
      }
      if (INITIAL_ICP_CONSTRAINT)
      {
         Aeq.reshape(Aeq.getNumRows() + 2, Aeq.getNumCols(), true);
         beq.reshape(beq.getNumRows() + 2, beq.getNumCols(), true);
      }

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
      CommonOps.scale(COP_WEIGHT / adjustmentSteps, FDtFD);

      if (graphicsRegistry != null)
      {
         for (int i = 0; i < icps.size(); i++)
         {
            YoArtifactPosition icpViz = new YoArtifactPosition("Icp" + i, (YoFramePoint2D) icps.get(i), GraphicType.BALL, Color.ORANGE, 0.002);
            graphicsRegistry.registerArtifact(getClass().getSimpleName(), icpViz);
         }
         for (int i = 0; i < cops.size(); i++)
         {
            YoArtifactPosition copViz = new YoArtifactPosition("Cop" + i, (YoFramePoint2D) cops.get(i), GraphicType.BALL, Color.GREEN, 0.002);
            graphicsRegistry.registerArtifact(getClass().getSimpleName(), copViz);
         }
         for (int i = 0; i < originalCops.size(); i++)
         {
            YoArtifactPosition copViz = new YoArtifactPosition("OriginalCop" + i, (YoFramePoint2D) originalCops.get(i), GraphicType.BALL, Color.BLUE, 0.002);
            graphicsRegistry.registerArtifact(getClass().getSimpleName(), copViz);
         }
         for (int i = 0; i < constraintPolygons.size(); i++)
         {
            YoArtifactPolygon constraintViz = new YoArtifactPolygon("ConstraintPolygon" + i, (YoFrameConvexPolygon2D) constraintPolygons.get(i), Color.BLUE, false);
            graphicsRegistry.registerArtifact(getClass().getSimpleName(), constraintViz);
            constraintPolygons.get(i).setToNaN();
         }
      }
      parentRegistry.addChild(registry);
   }

   public void setOmega(double omega)
   {
      this.omega = omega;

      // Initialize matrices that will remain constant of omega is unchanged:
      double alpha = 1.0 + timestep * omega;
      for (int i = 0; i < adjustmentSteps; i++)
      {
         AIcp.set(0, 2 * i, timestep * omega * Math.pow(alpha, adjustmentSteps - i - 1));
         AIcp.set(1, 2 * i + 1, timestep * omega * Math.pow(alpha, adjustmentSteps - i - 1));
      }
      CommonOps.multTransA(AIcp, WIcp, AtWicp);
      Q.set(FDtFD);
      CommonOps.multAdd(AtWicp, AIcp, Q);
   }

   public void setCopTrajectory(ObjDoubleConsumer<Point2DBasics> copTrajectory)
   {
      setCopTrajectory(copTrajectory, 0.0);
   }

   public void setCopTrajectory(ObjDoubleConsumer<Point2DBasics> copTrajectory, double timeInSequence)
   {
      // Re-sample the trajectory at the sample rate of this planner:
      for (int i = 0; i < cops.size(); i++)
      {
         double time = timestep * i + timeInSequence;
         copTrajectory.accept(cops.get(i), time);
         if (i < originalCops.size())
            originalCops.get(i).set(cops.get(i));
      }

      // Initialize the final ICP to match the final CoP:
      icps.get(icps.size() - 1).set(cops.get(cops.size() - 1));
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

   public void setInitialIcp(Point2DReadOnly initialIcp)
   {
      this.initialIcp.set(initialIcp);
   }

   public void setCopConstraints(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes)
   {
      setCopConstraints(supportPolygons, supportTimes, 0.0);
   }

   public void setCopConstraints(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes, double timeInSequence)
   {
      bin.reshape(0, 1);
      Ain.reshape(0, 2 * adjustmentSteps);
      Arrays.fill(Ain.data, 0.0);
      int index = 0;
      for (int step = 0; step < adjustmentSteps; step++)
      {
         double time = timestep * step + timeInSequence;
         while (index < supportTimes.size() - 1 && time > supportTimes.get(index + 1))
            index++;

         ConvexPolygon2DReadOnly supportPolygon = supportPolygons.get(index);
         PolygonWiggler.convertToInequalityConstraints(supportPolygon, subAin, subbin, COP_SAFETY_DISTANCE);

         int currentConstraints = bin.getNumRows();
         int newConstraints = subbin.getNumRows();
         bin.reshape(currentConstraints + newConstraints, 1, true);
         Ain.reshape(currentConstraints + newConstraints, Ain.getNumCols(), true);
         CommonOps.insert(subAin, Ain, currentConstraints, 2 * step);
         CommonOps.insert(subbin, bin, currentConstraints, 0);

         constraintPolygons.get(step).set(supportPolygon);
      }
   }

   public void compute()
   {
      reverseTimeIntegrate(icps.size() - 2, adjustmentSteps);
      solveInitialCopTrajectory();
      reverseTimeIntegrate(adjustmentSteps - 1, 0);
   }

   private void solveInitialCopTrajectory()
   {
      // In this case there is nothing to adjust since the initial and final CoP location are constrained
      if (adjustmentSteps <= 1)
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
      double alpha = 1.0 + timestep * omega;
      double factor = Math.pow(alpha, adjustmentSteps);
      bIcp.set(0, 0, icps.get(adjustmentSteps).getX() - (initialIcp.getX() * factor - amY));
      bIcp.set(1, 0, icps.get(adjustmentSteps).getY() - (initialIcp.getY() * factor + amX));
      NativeCommonOps.mult(AtWicp, bIcp, f);

      // Setup equality constraints
      beq.set(0, cops.get(adjustmentSteps - 1).getX());
      beq.set(1, cops.get(adjustmentSteps - 1).getY());
      int additionalConstraintOffset = 2;
      if (INITIAL_COP_CONSTRAINT)
      {
         beq.set(additionalConstraintOffset, cops.get(0).getX());
         beq.set(additionalConstraintOffset + 1, cops.get(0).getY());
         additionalConstraintOffset += 2;
      }
      if (INITIAL_ICP_CONSTRAINT)
      {
         CommonOps.insert(AIcp, Aeq, additionalConstraintOffset, 0);
         beq.set(additionalConstraintOffset, -bIcp.get(0));
         beq.set(additionalConstraintOffset + 1, -bIcp.get(1));
         additionalConstraintOffset += 2;
      }

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

         icp.setX((nextIcp.getX() + omega * timestep * (cop.getX() + angularMomentum.getY())) / (1.0 + omega * timestep));
         icp.setY((nextIcp.getY() + omega * timestep * (cop.getY() - angularMomentum.getX())) / (1.0 + omega * timestep));
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
