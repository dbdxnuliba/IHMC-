package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollinearForceBasedPlannerOptimizationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoInteger numberOfSupportPolygonConstraintsPerSegment;
   private final YoInteger numberOfCoMPositionConstraintsPerSegment;
   private final YoInteger numberOfDynamicsConstraintsPerSegment;
   private final YoInteger numberOfScalarConstraintsPerSegment;

   private final FrameVector3DReadOnly gravity;
   private final CollinearForceBasedPlannerIterationResult sqpSolution;

   private final FramePoint3D initialCoMPosition = new FramePoint3D();
   private final FramePoint3D finalCoMPosition = new FramePoint3D();
   private final FramePoint3D initialCoPPosition = new FramePoint3D();
   private final FramePoint3D finalCoPPosition = new FramePoint3D();
   private final FrameVector3D initialCoMVelocity = new FrameVector3D();
   private final FrameVector3D finalCoMVelocity = new FrameVector3D();

   private final DenseMatrix64F solver_objH;
   private final DenseMatrix64F solver_objf;
   private final DenseMatrix64F solver_conAin;
   private final DenseMatrix64F solver_conbin;
   private final DenseMatrix64F solver_conAeq;
   private final DenseMatrix64F solver_conbeq;
   private final DenseMatrix64F solver_ub;
   private final DenseMatrix64F solver_lb;
   private final DenseMatrix64F solver_qpSoln;

   private final ActiveSetQPSolver qpSolver;

   public CollinearForceBasedPlannerOptimizationControlModule(CollinearForceBasedPlannerIterationResult sqpSolution, FrameVector3DReadOnly gravity,
                                                              YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();

      numberOfCoMPositionConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfCoMPositionConstraintsPerSegment", registry);
      numberOfSupportPolygonConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfSupportPolygonConstraintsPerSegment", registry);
      numberOfDynamicsConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfDynamicsConstraintsPerSegment", registry);
      numberOfScalarConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfScalarConstraintsPerSegment", registry);

      this.sqpSolution = sqpSolution;
      this.gravity = gravity;

      solver_objH = new DenseMatrix64F(0, 1);
      solver_objf = new DenseMatrix64F(0, 1);
      solver_conAin = new DenseMatrix64F(0, 1);
      solver_conbin = new DenseMatrix64F(0, 1);
      solver_conAeq = new DenseMatrix64F(0, 1);
      solver_conbeq = new DenseMatrix64F(0, 1);
      solver_lb = new DenseMatrix64F(0, 1);
      solver_ub = new DenseMatrix64F(0, 1);
      solver_qpSoln = new DenseMatrix64F(0, 1);

      qpSolver = new JavaQuadProgSolver();
   }

   public void initialize(CollinearForcePlannerParameters parameters)
   {

   }

   public void reset()
   {

   }

   public void setInitialState(YoFramePoint initialCoMLocation, YoFramePoint initialCoPLocation, YoFrameVector initialCoMVelocity)
   {
      this.initialCoMPosition.setIncludingFrame(initialCoMLocation);
      this.initialCoMVelocity.setIncludingFrame(initialCoMVelocity);
      this.initialCoPPosition.setIncludingFrame(initialCoPLocation);
   }

   public void setFinalState(YoFramePoint finalCoMLocation, YoFramePoint finalCoPLocation, YoFrameVector finalCoMVelocity)
   {
      this.finalCoMPosition.setIncludingFrame(finalCoMLocation);
      this.finalCoMVelocity.setIncludingFrame(finalCoMVelocity);
      this.finalCoPPosition.setIncludingFrame(finalCoPLocation);
   }

   public void generateLinearizedSystemObjective()
   {

   }

   public void generateLinearizedSystemConstraints()
   {

   }

   public void updateSolution()
   {

   }

   public boolean compute()
   {
      return false;
   }
}
