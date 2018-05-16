package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import afu.org.checkerframework.checker.units.qual.A;
import us.ihmc.commonWalkingControlModules.controlModules.flight.BipedContactType;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;
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
   private final YoInteger numberOfCoMTrajectoryCoefficients;
   private final YoInteger numberOfCoPTrajectoryCoefficients;
   private final YoInteger numberOfScalarTrajectoryCoefficients;
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
   private final ConstraintMatrixHandler inequalityConstraintHandler;
   private final ConstraintMatrixHandler equalityConstraintHandler;
   private final DenseMatrix64F solver_qpSoln;

   private final ActiveSetQPSolver qpSolver;

   private final ConstraintGenerationHelper constraintGenerationHelper;
   private List<CollinearForceMotionPlannerSegment> segmentList;
   private final Axis[] copAxis = new Axis[] {Axis.X, Axis.Y};
   private final Axis[] comAxis = Axis.values;

   public CollinearForceBasedPlannerOptimizationControlModule(CollinearForceBasedPlannerIterationResult sqpSolution, YoInteger numberOfPlanningSegments,
                                                              FrameVector3DReadOnly gravity, YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();

      numberOfCoMPositionConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfCoMPositionConstraintsPerSegment", registry);
      numberOfSupportPolygonConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfSupportPolygonConstraintsPerSegment", registry);
      numberOfDynamicsConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfDynamicsConstraintsPerSegment", registry);
      numberOfScalarConstraintsPerSegment = new YoInteger(namePrefix + "NumberOfScalarConstraintsPerSegment", registry);
      maxDegreeOfCoMSmoothnessConstraints = new YoInteger(namePrefix + "MaxDegreeCoMSmoothnessConstraint", registry);
      maxDegreeOfCoPSmoothnessConstraints = new YoInteger(namePrefix + "MaxDegreeCoPSmoothnessConstraint", registry);
      maxDegreeOfScalarSmoothnessConstraints = new YoInteger(namePrefix + "MaxDegreeScalarSmoothnessConstraint", registry);

      // AS: These are YoVariablized for logging and hard coded as anything else will under/over constrain the QP.
      maxDegreeOfCoMSmoothnessConstraints.set(1);
      maxDegreeOfCoPSmoothnessConstraints.set(1);
      maxDegreeOfScalarSmoothnessConstraints.set(1);

      this.numberOfSegments = numberOfPlanningSegments;
      // AS: Also yovariablized to for logging
      numberOfCoMTrajectoryCoefficients = new YoInteger(namePrefix + "NumberOfCoMTrajectoryCoefficients", registry);
      numberOfCoMTrajectoryCoefficients.set(CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients);
      numberOfCoPTrajectoryCoefficients = new YoInteger(namePrefix + "NumberOfCoPTrajectoryCoefficients", registry);
      numberOfCoPTrajectoryCoefficients.set(CollinearForceBasedCoMMotionPlanner.numberOfCoPTrajectoryCoefficients);
      numberOfScalarTrajectoryCoefficients = new YoInteger(namePrefix + "NumberOfScalarTrajectoryCoefficients", registry);
      numberOfScalarTrajectoryCoefficients.set(CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients);
      this.sqpSolution = sqpSolution;
      this.gravity = gravity;

      equalityConstraintHandler = new ConstraintMatrixHandler(numberOfPlanningSegments, numberOfCoMTrajectoryCoefficients, numberOfCoPTrajectoryCoefficients,
                                                              numberOfScalarTrajectoryCoefficients);
      inequalityConstraintHandler = new ConstraintMatrixHandler(numberOfPlanningSegments, numberOfCoMTrajectoryCoefficients, numberOfCoPTrajectoryCoefficients,
                                                                numberOfScalarTrajectoryCoefficients);
      constraintGenerationHelper = new ConstraintGenerationHelper();

      solver_objH = new DenseMatrix64F(0, 1);
      solver_objf = new DenseMatrix64F(0, 1);
      solver_qpSoln = new DenseMatrix64F(0, 1);

      qpSolver = new JavaQuadProgSolver();
   }

   public void initialize(CollinearForcePlannerParameters parameters)
   {

   }

   public void reset()
   {
      segmentList = null;
      equalityConstraintHandler.reshape();
      inequalityConstraintHandler.reshape();
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
      reshapeMatrices();
      generateCoMSmoothnessConstraints();
      generateCoPSmoothnessConstraints();
      generateScalarSmoothnessConstraints();
      generateCoPLocationConstraintsFromContactStates();
      generateCoMLocationConstraintsFromContactStates();
      generateScalarConstraintsFromContactStates();
      generateInitialFinalCoMLocationConstraintsFromDesireds();
      generateInitialFinalCoPLocationConstraintsFromDesireds();
      generateInitialFinalScalarConstraintsFromDesireds();
      generateDynamicsConstraintsForSegments();
      generateAccelerationMinimizationObjective();
      submitQPMatricesAndRunOptimization();
      return false;
   }

   private void reshapeMatrices()
   {
      numberOfSegments.set(segmentList.size());
      equalityConstraintHandler.reshape();
      inequalityConstraintHandler.reshape();
   }

   private void generateScalarConstraintsFromContactStates()
   {
      List<Trajectory> scalarTrajectories = sqpSolution.scalarProfile;
      for (int i = 0; i < segmentList.size(); i++)
      {
         ContactState contactState = segmentList.get(i).getContactState();
         List<Double> nodeTimes = generateScalarLimitConstraintNodeTimesForSegment(segmentList.get(i).getSegmentDuration());
         scalarTrajectories.get(i).getCoefficientVector(tempA);;
         constraintGenerationHelper.generateScalarConstraintMatrix(tempJ1, tempC1, tempA, numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1, nodeTimes);
         if(contactState.getContactType().isRobotSupported())
         {
            
         }
         else
         {
            
         }
      }
   }

   private List<Double> generateScalarLimitConstraintNodeTimesForSegment(double segmentDuration)
   {
      tempDoubleList.clear();
      int numberOfConstraints = numberOfScalarConstraintsPerSegment.getIntegerValue();
      double dt = segmentDuration / (numberOfConstraints - 1);
      for (int i = 0; i < numberOfScalarConstraintsPerSegment.getIntegerValue(); i++)
         tempDoubleList.add(dt * i);
      return tempDoubleList;
   }

   private void generateCoMLocationConstraintsFromContactStates()
   {
      for (int i = 0; i < segmentList.size(); i++)
      {
         
      }
   }

   private void generateCoPLocationConstraintsFromContactStates()
   {
      
   }

   public void generateScalarSmoothnessConstraints()
   {
      List<Trajectory> scalarTrajectories = sqpSolution.scalarProfile;
      int scalarTrajectoryOrder = this.numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1;
      for (int i = 0; i < numberOfSegments.getIntegerValue() - 1; i++)
      {
         double segmentDuration = segmentList.get(i).getSegmentDuration();
         Trajectory segment = scalarTrajectories.get(i);
         segment.getCoefficientVector(tempA);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, scalarTrajectoryOrder,
                                                                                maxDegreeOfScalarSmoothnessConstraints.getIntegerValue(), segmentDuration);
         Trajectory nextSegment = scalarTrajectories.get(i + 1);
         nextSegment.getCoefficientVector(tempA);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC2, tempA, scalarTrajectoryOrder,
                                                                                maxDegreeOfScalarSmoothnessConstraints.getIntegerValue(), segmentDuration);
         equalityConstraintHandler.addInterSegmentScalarConstraint(i, tempJ1, tempC1, i + 1, tempJ2, tempC2);

      }
   }

   public void generateCoPSmoothnessConstraints()
   {
      List<Trajectory3D> copTrajectories = sqpSolution.copTrajectories;
      int copTrajectoryOrder = this.numberOfCoPTrajectoryCoefficients.getIntegerValue() - 1;
      for (int i = 0; i < numberOfSegments.getIntegerValue() - 1; i++)
      {
         double segmentDuration = segmentList.get(i).getSegmentDuration();
         Trajectory3D segment = copTrajectories.get(i);
         Trajectory3D nextSegment = copTrajectories.get(i + 1);
         for (Axis axis : copAxis)
         {
            Trajectory axisSegmentTrajectory = segment.getTrajectory(axis);
            axisSegmentTrajectory.getCoefficientVector(tempA);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, copTrajectoryOrder,
                                                                                   maxDegreeOfCoPSmoothnessConstraints.getIntegerValue(), segmentDuration);
            Trajectory axisNextSegmentTrajectory = nextSegment.getTrajectory(axis);
            axisNextSegmentTrajectory.getCoefficientVector(tempA);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC2, tempA, copTrajectoryOrder,
                                                                                   maxDegreeOfCoPSmoothnessConstraints.getIntegerValue(), segmentDuration);
            equalityConstraintHandler.addInterSegmentCoPConstraint(axis, i, tempJ1, tempC1, i + 1, tempJ2, tempC2);
         }
      }
   }

   public void generateCoMSmoothnessConstraints()
   {
      List<Trajectory3D> comTrajectories = sqpSolution.comTrajectories;
      int comTrajectoryOrder = this.numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1;
      for (int i = 0; i < numberOfSegments.getIntegerValue() - 1; i++)
      {
         double segmentDuration = segmentList.get(i).getSegmentDuration();
         Trajectory3D segment = comTrajectories.get(i);
         Trajectory3D nextSegment = comTrajectories.get(i + 1);
         for (Axis axis : comAxis)
         {
            Trajectory axisSegmentTrajectory = segment.getTrajectory(axis);
            axisSegmentTrajectory.getCoefficientVector(tempA);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, comTrajectoryOrder,
                                                                                   maxDegreeOfCoMSmoothnessConstraints.getIntegerValue(), segmentDuration);
            Trajectory axisNextSegmentTrajectory = nextSegment.getTrajectory(axis);
            axisNextSegmentTrajectory.getCoefficientVector(tempA);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC2, tempA, comTrajectoryOrder,
                                                                                   maxDegreeOfCoMSmoothnessConstraints.getIntegerValue(), segmentDuration);
            equalityConstraintHandler.addInterSegmentCoMConstraint(axis, i, tempJ1, tempC1, i + 1, tempJ2, tempC2);
         }
      }
   }

   public String toString()
   {
      String str = "Min: \n" + "Subject to: " + equalityConstraintHandler.getCoefficientMatrix().toString() + " "
            + equalityConstraintHandler.getBiasMatrix().toString();
      return str;
   }

   private void generateInitialFinalScalarConstraintsFromDesireds()
   {
      double initialOmega = -gravity.getZ() / desiredInitialCoMPosition.getZ();
      Trajectory firstScalarSegment = sqpSolution.scalarProfile.get(0);
      firstScalarSegment.getCoefficientVector(tempA);
      constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, firstScalarSegment.getNumberOfCoefficients() - 1, 0, 0.0);
      CommonOps.subtract(initialOmega, tempC1, tempC1);
      equalityConstraintHandler.addIntraSegmentScalarConstraint(0, tempJ1, tempC1);

      double finalOmega = -gravity.getZ() / desiredFinalCoMPosition.getZ();
      int lastSegmentIndex = numberOfSegments.getIntegerValue() - 1;
      Trajectory lastScalarSegment = sqpSolution.scalarProfile.get(lastSegmentIndex);
      lastScalarSegment.getCoefficientVector(tempA);
      constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, lastScalarSegment.getNumberOfCoefficients() - 1, 0,
                                                                             lastScalarSegment.getFinalTime());
      CommonOps.subtract(finalOmega, tempC1, tempC1);
      equalityConstraintHandler.addIntraSegmentScalarConstraint(lastSegmentIndex, tempJ1, tempC1);
   }

   private void generateInitialFinalCoPLocationConstraintsFromDesireds()
   {
      Trajectory3D firstCoPSegment = sqpSolution.copTrajectories.get(0);
      int lastSegmentIndex = numberOfSegments.getIntegerValue() - 1;
      Trajectory3D lastCoPSegment = sqpSolution.copTrajectories.get(lastSegmentIndex);
      for (Axis axis : copAxis)
      {
         double desiredInitalValue = desiredInitialCoPPosition.getElement(axis.ordinal());
         Trajectory firstAxisSegment = firstCoPSegment.getTrajectory(axis);
         firstAxisSegment.getCoefficientVector(tempA);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, firstAxisSegment.getNumberOfCoefficients() - 1, 0, 0.0);
         CommonOps.subtract(desiredInitalValue, tempC1, tempC1);
         equalityConstraintHandler.addIntraSegmentCoPConstraint(axis, 0, tempJ1, tempC1);

         double desiredFinalValue = desiredFinalCoPPosition.getElement(axis.ordinal());
         Trajectory lastAxisSegment = lastCoPSegment.getTrajectory(axis);
         lastAxisSegment.getCoefficientVector(tempA);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, lastAxisSegment.getNumberOfCoefficients() - 1, 0,
                                                                                lastAxisSegment.getFinalTime());
         CommonOps.subtract(desiredFinalValue, tempC1, tempC1);
         equalityConstraintHandler.addIntraSegmentCoPConstraint(axis, lastSegmentIndex, tempJ1, tempC1);
      }
   }

   private void generateInitialFinalCoMLocationConstraintsFromDesireds()
   {
      Trajectory3D firstCoMSegment = sqpSolution.comTrajectories.get(0);
      int lastSegmentIndex = numberOfSegments.getIntegerValue() - 1;
      Trajectory3D lastCoMSegment = sqpSolution.comTrajectories.get(lastSegmentIndex);
      for (Axis axis : comAxis)
      {
         tempC2.reshape(2, 1);
         tempC2.set(0, 0, desiredInitialCoMPosition.getElement(axis.ordinal()));
         tempC2.set(1, 0, desiredInitialCoMVelocity.getElement(axis.ordinal()));
         Trajectory firstAxisSegment = firstCoMSegment.getTrajectory(axis);
         firstAxisSegment.getCoefficientVector(tempA);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA, firstAxisSegment.getNumberOfCoefficients() - 1, 1, 0.0);
         CommonOps.subtractEquals(tempC2, tempC1);
         equalityConstraintHandler.addIntraSegmentCoMConstraint(axis, 0, tempJ1, tempC2);

         tempC2.set(0, 0, desiredFinalCoMPosition.getElement(axis.ordinal()));
         tempC2.set(1, 0, desiredFinalCoMVelocity.getElement(axis.ordinal()));
         Trajectory lastAxisSegment = lastCoMSegment.getTrajectory(axis);
         lastAxisSegment.getCoefficientVector(tempA);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC1, tempA, lastAxisSegment.getNumberOfCoefficients() - 1, 1,
                                                                                lastAxisSegment.getFinalTime());
         CommonOps.subtractEquals(tempC2, tempC1);
         equalityConstraintHandler.addIntraSegmentCoMConstraint(axis, lastSegmentIndex, tempJ1, tempC2);
      }
   }

   private final DenseMatrix64F comConstraints = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F copConstraints = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F scalarConstraints = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F constraintViolation = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F comCoefficients = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F copCoefficients = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F scalarCoefficients = new DenseMatrix64F(0, 1);
   private final List<Double> tempDoubleList = new ArrayList<>();

   private void generateDynamicsConstraintsForSegments()
   {
      for (int i = 0; i < numberOfSegments.getIntegerValue(); i++)
      {
         Trajectory3D comTrajectory = sqpSolution.comTrajectories.get(i);
         Trajectory3D copTrajectory = sqpSolution.copTrajectories.get(i);
         Trajectory scalarTrajectory = sqpSolution.scalarProfile.get(i);
         List<Double> nodeTimesList = generateDynamicsCollocationConstraintNodeTimesForSegment(segmentList.get(i).getSegmentDuration());
         // TODO include Z axis in loop after getting the Z profile from the contact state
         for (Axis axis : copAxis)
         {
            Trajectory axisCoMTrajectory = comTrajectory.getTrajectory(axis);
            Trajectory axisCoPTrajectory = copTrajectory.getTrajectory(axis);
            axisCoMTrajectory.getCoefficientVector(comCoefficients);
            axisCoPTrajectory.getCoefficientVector(copCoefficients);
            scalarTrajectory.getCoefficientVector(scalarCoefficients);
            constraintGenerationHelper.generateDynamicsCollocationConstraints(comConstraints, copConstraints, scalarConstraints, constraintViolation,
                                                                              nodeTimesList, comCoefficients, copCoefficients, scalarCoefficients,
                                                                              numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1,
                                                                              numberOfCoPTrajectoryCoefficients.getIntegerValue() - 1,
                                                                              numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1, gravity.getElement(axis.ordinal()));
            equalityConstraintHandler.addIntraSegmentMultiQuantityConstraints(axis, i, comConstraints, copConstraints, scalarConstraints, constraintViolation);
         }
         Trajectory zCoMTrajectory = comTrajectory.getTrajectory(Axis.Z);
         zCoMTrajectory.getCoefficientVector(comCoefficients);
         copCoefficients.reshape(1, numberOfCoPTrajectoryCoefficients.getIntegerValue());
         copCoefficients.zero();
         scalarTrajectory.getCoefficientVector(scalarCoefficients);
         constraintGenerationHelper.generateDynamicsCollocationConstraints(comConstraints, copConstraints, scalarConstraints, constraintViolation,
                                                                           nodeTimesList, comCoefficients, copCoefficients, scalarCoefficients,
                                                                           numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1,
                                                                           numberOfCoPTrajectoryCoefficients.getIntegerValue() - 1,
                                                                           numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1, gravity.getElement(Axis.Z.ordinal()));
         equalityConstraintHandler.addIntraSegmentMultiQuantityConstraintsForZAxis(i, comConstraints, scalarConstraints, constraintViolation);
      }
   }

   private List<Double> generateDynamicsCollocationConstraintNodeTimesForSegment(double segmentDuration)
   {
      tempDoubleList.clear();
      int numberOfConstraints = numberOfDynamicsConstraintsPerSegment.getIntegerValue();
      double dt = segmentDuration / (numberOfConstraints - 1);
      for (int i = 0; i < numberOfDynamicsConstraintsPerSegment.getIntegerValue(); i++)
         tempDoubleList.add(dt * i);
      return tempDoubleList;
   }

   private void submitQPMatricesAndRunOptimization()
   {

   }
}