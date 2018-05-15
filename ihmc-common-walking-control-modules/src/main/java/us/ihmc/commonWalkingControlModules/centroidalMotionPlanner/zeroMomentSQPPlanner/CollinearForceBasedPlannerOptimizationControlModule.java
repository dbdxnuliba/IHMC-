package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

// TODO Optimize generation of smoothness constraints
public class CollinearForceBasedPlannerOptimizationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoInteger numberOfSupportPolygonConstraintsPerSegment;
   private final YoInteger numberOfCoMPositionConstraintsPerSegment;
   private final YoInteger numberOfDynamicsConstraintsPerSegment;
   private final YoInteger numberOfScalarConstraintsPerSegment;
   private final YoInteger maxDegreeOfCoMSmoothnessConstraints;
   private final YoInteger maxDegreeOfCoPSmoothnessConstraints;
   private final YoInteger maxDegreeOfScalarSmoothnessConstraints;

   private final YoInteger numberOfSegments;
   private final FrameVector3DReadOnly gravity;
   private final CollinearForceBasedPlannerIterationResult sqpSolution;

   private final FramePoint3D desiredInitialCoMPosition = new FramePoint3D();
   private final FramePoint3D desiredFinalCoMPosition = new FramePoint3D();
   private final FramePoint3D desiredInitialCoPPosition = new FramePoint3D();
   private final FramePoint3D desiredFinalCoPPosition = new FramePoint3D();
   private final FrameVector3D desiredInitialCoMVelocity = new FrameVector3D();
   private final FrameVector3D desiredFinalCoMVelocity = new FrameVector3D();

   private final DenseMatrix64F solver_objH;
   private final DenseMatrix64F solver_objf;
   private final DenseMatrix64F solver_conAeq;
   private final DenseMatrix64F solver_conbeq;
   private final DenseMatrix64F solver_conAin;
   private final DenseMatrix64F solver_conbin;
   private final DenseMatrix64F solver_ub;
   private final DenseMatrix64F solver_lb;
   private final DenseMatrix64F solver_qpSoln;

   private final ActiveSetQPSolver qpSolver;

   private final ConstraintGenerator constraintGenerator;
   private final ConstraintMatrixHandler constraintMatrixHandler;
   private List<CollinearForceMotionPlannerSegment> segmentList;

   public CollinearForceBasedPlannerOptimizationControlModule(CollinearForceBasedPlannerIterationResult sqpSolution, YoInteger numberOfPlanningSegments,
                                                              FrameVector3DReadOnly gravity, YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();

      numberOfCoMPositionConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfCoMPositionConstraintsPerSegment", registry);
      numberOfSupportPolygonConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfSupportPolygonConstraintsPerSegment", registry);
      numberOfDynamicsConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfDynamicsConstraintsPerSegment", registry);
      numberOfScalarConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfScalarConstraintsPerSegment", registry);
      maxDegreeOfCoMSmoothnessConstraints = new YoInteger(namePrefix + "MaxDegreeCoMSmoothnessConstraint", registry);
      maxDegreeOfCoPSmoothnessConstraints = new YoInteger(namePrefix + "MaxDegreeCoMSmoothnessConstraint", registry);
      maxDegreeOfScalarSmoothnessConstraints = new YoInteger(namePrefix + "MaxDegreeCoMSmoothnessConstraint", registry);

      // AS: These are YoVariablized for logging and hard coded as anything else will under/over constrain the QP.
      maxDegreeOfCoMSmoothnessConstraints.set(2);
      maxDegreeOfCoPSmoothnessConstraints.set(2);
      maxDegreeOfScalarSmoothnessConstraints.set(2);

      this.numberOfSegments = numberOfPlanningSegments;
      this.sqpSolution = sqpSolution;
      this.gravity = gravity;

      constraintMatrixHandler = new ConstraintMatrixHandler(numberOfPlanningSegments, CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients,
                                                            CollinearForceBasedCoMMotionPlanner.numberOfCoPTrajectoryCoefficients,
                                                            CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients);
      constraintGenerator = new ConstraintGenerator(constraintMatrixHandler);

      solver_objH = new DenseMatrix64F(0, 1);
      solver_objf = new DenseMatrix64F(0, 1);
      solver_conAeq = new DenseMatrix64F(0, 1);
      solver_conbeq = new DenseMatrix64F(0, 1);
      solver_conAin = new DenseMatrix64F(0, 1);
      solver_conbin = new DenseMatrix64F(0, 1);
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
      segmentList = null;
   }

   public void setDesiredInitialState(YoFramePoint initialCoMLocation, YoFramePoint initialCoPLocation, YoFrameVector initialCoMVelocity)
   {
      this.desiredInitialCoMPosition.setIncludingFrame(initialCoMLocation);
      this.desiredInitialCoMVelocity.setIncludingFrame(initialCoMVelocity);
      this.desiredInitialCoPPosition.setIncludingFrame(initialCoPLocation);
   }

   public void setDesiredFinalState(YoFramePoint finalCoMLocation, YoFramePoint finalCoPLocation, YoFrameVector finalCoMVelocity)
   {
      this.desiredFinalCoMPosition.setIncludingFrame(finalCoMLocation);
      this.desiredFinalCoMVelocity.setIncludingFrame(finalCoMVelocity);
      this.desiredFinalCoPPosition.setIncludingFrame(finalCoPLocation);
   }

   private void generateAccelerationMinimizationObjective()
   {

   }

   public void submitSegmentList(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      this.segmentList = segmentList;
   }

   private final DenseMatrix64F tempJ1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempC1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempA = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempJ2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempC2 = new DenseMatrix64F(0, 1);

   public void updateSolution()
   {

   }

   public boolean compute()
   {
      generateCoMSmoothnessConstraints();
      generateCoPSmoothnessConstraints();
      generateScalarSmoothnessConstraints();
      generateCoPLocationConstraintsFromContactStates(segmentList);
      generateCoMLocationConstraintsFromContactStates(segmentList);
      generateScalarConstraintsFromContactStates(segmentList);
      generateInitialFinalCoMLocationConstraintsFromDesireds();
      generateInitialFinalCoPLocationConstraintsFromDesireds();
      generateInitialFinalScalarConstraintsFromDesireds();
      generateDynamicsConstraintsForSegments();
      generateAccelerationMinimizationObjective();
      submitQPMatricesAndRunOptimization();
      return false;
   }

   private void generateScalarConstraintsFromContactStates(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      // TODO Auto-generated method stub
      
   }

   private void generateCoMLocationConstraintsFromContactStates(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      // TODO Auto-generated method stub
      
   }

   private void generateCoPLocationConstraintsFromContactStates(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      // TODO Auto-generated method stub
      
   }

   private void generateScalarSmoothnessConstraints()
   {
      // TODO Auto-generated method stub
      
   }

   private void generateCoPSmoothnessConstraints()
   {
      // TODO Auto-generated method stub
      
   }

   private void generateCoMSmoothnessConstraints()
   {
      constraintGenerator.generateSmoothnessConstraint();
      for (int i = 0; i < numberOfSegments.getIntegerValue(); i++)
      {
      }
   }

   private void generateInitialFinalScalarConstraintsFromDesireds()
   {
      // TODO Auto-generated method stub

   }

   private void generateInitialFinalCoPLocationConstraintsFromDesireds()
   {
      // TODO Auto-generated method stub

   }

   private void generateInitialFinalCoMLocationConstraintsFromDesireds()
   {
      // TODO Auto-generated method stub

   }

   private void generateDynamicsConstraintsForSegments()
   {
      // TODO Auto-generated method stub

   }
   
   private void submitQPMatricesAndRunOptimization()
   {

   }
}