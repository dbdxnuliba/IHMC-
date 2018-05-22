package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

// TODO Optimize generation of constraints
public class CollinearForceBasedPlannerOptimizationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoInteger errorCode;
   private final YoInteger numberOfSupportPolygonConstraintsPerSegment;
   private final YoInteger numberOfCoMPositionConstraintsPerSegment;
   private final YoInteger numberOfDynamicsConstraintsPerSegment;
   private final YoInteger numberOfScalarConstraintsPerSegment;
   private final YoInteger maxDegreeOfCoMSmoothnessConstraints;
   private final YoInteger maxDegreeOfCoPSmoothnessConstraints;
   private final YoInteger maxDegreeOfScalarSmoothnessConstraints;
   private final YoDouble comSupportPolygonXYConstraintOffset;
   private final YoDouble comZMaxHeightConstraint;
   private final YoDouble comZMinHeightConstraint;

   private final YoInteger numberOfSegments;
   private final YoInteger numberOfCoMTrajectoryCoefficients;
   private final YoInteger numberOfCoPTrajectoryCoefficients;
   private final YoInteger numberOfScalarTrajectoryCoefficients;
   private final FrameVector3DReadOnly gravity;
   private final CollinearForceBasedPlannerResult sqpSolution;

   private final FramePoint3D desiredInitialCoMPosition = new FramePoint3D();
   private final FramePoint3D desiredFinalCoMPosition = new FramePoint3D();
   private final FramePoint3D desiredInitialCoPPosition = new FramePoint3D();
   private final FramePoint3D desiredFinalCoPPosition = new FramePoint3D();
   private final FrameVector3D desiredInitialCoMVelocity = new FrameVector3D();
   private final FrameVector3D desiredFinalCoMVelocity = new FrameVector3D();

   private final DenseMatrix64F solver_objH;
   private final DenseMatrix64F solver_objf;
   private final DenseMatrix64F solver_conUb;
   private final DenseMatrix64F solver_conLb;
   private final ConstraintMatrixHandler inequalityConstraintHandler;
   private final ConstraintMatrixHandler equalityConstraintHandler;
   private final DenseMatrix64F solver_qpSoln;

   private final ActiveSetQPSolver qpSolver;

   private final CollinearForcePlannerOptimizationControlModuleHelper constraintGenerationHelper;
   private List<CollinearForceMotionPlannerSegment> segmentList;
   private final Axis[] copAxis = new Axis[] {Axis.X, Axis.Y};
   private final Axis[] comAxis = Axis.values;

   private final ConvexPolygonScaler polygonScaler;

   public CollinearForceBasedPlannerOptimizationControlModule(CollinearForceBasedPlannerResult sqpSolution, YoInteger numberOfPlanningSegments,
                                                              FrameVector3DReadOnly gravity, YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();

      errorCode = new YoInteger(namePrefix + "ErrorCode", registry);
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
      comSupportPolygonXYConstraintOffset = new YoDouble(namePrefix + "CoMSupportPolygonConstraintOffset", registry);
      comZMaxHeightConstraint = new YoDouble(namePrefix + "CoMZMaxHeight", registry);
      comZMinHeightConstraint = new YoDouble(namePrefix + "CoMZMinHeight", registry);
      this.sqpSolution = sqpSolution;
      this.gravity = gravity;

      equalityConstraintHandler = new ConstraintMatrixHandler(numberOfPlanningSegments, numberOfCoMTrajectoryCoefficients, numberOfCoPTrajectoryCoefficients,
                                                              numberOfScalarTrajectoryCoefficients);
      inequalityConstraintHandler = new ConstraintMatrixHandler(numberOfPlanningSegments, numberOfCoMTrajectoryCoefficients, numberOfCoPTrajectoryCoefficients,
                                                                numberOfScalarTrajectoryCoefficients);
      constraintGenerationHelper = new CollinearForcePlannerOptimizationControlModuleHelper();
      polygonScaler = new ConvexPolygonScaler();

      solver_objH = new DenseMatrix64F(0, 1);
      solver_objf = new DenseMatrix64F(0, 1);
      solver_conUb = new DenseMatrix64F(0, 1);
      solver_conLb = new DenseMatrix64F(0, 1);
      solver_qpSoln = new DenseMatrix64F(0, 1);

      qpSolver = new JavaQuadProgSolver();
   }

   public void initialize(CollinearForcePlannerParameters parameters)
   {
      numberOfDynamicsConstraintsPerSegment.set(parameters.getNumberOfCollocationConstraintsPerSegment());
      numberOfCoMPositionConstraintsPerSegment.set(parameters.getNumberOfCoMPositionConstraintsPerSegment());
      numberOfSupportPolygonConstraintsPerSegment.set(parameters.getNumberOfSupportPolygonConstraintsPerSegment());
      comSupportPolygonXYConstraintOffset.set(parameters.getSupportPolygonMXYOffsetForCoMConstraint());
      comZMaxHeightConstraint.set(parameters.getMaxZHeight());
      comZMinHeightConstraint.set(parameters.getMinZHeight());
   }

   public void reset()
   {
      segmentList = null;
      equalityConstraintHandler.reshape();
      inequalityConstraintHandler.reshape();
      errorCode.set(0);
   }

   public void setDesiredInitialState(YoFramePoint initialCoMLocation, YoFramePoint initialCoPLocation, YoFrameVector initialCoMVelocity)
   {
      this.desiredInitialCoMPosition.setIncludingFrame(initialCoMLocation);
      this.desiredInitialCoMVelocity.setIncludingFrame(initialCoMVelocity);
      this.desiredInitialCoPPosition.setIncludingFrame(initialCoPLocation);
   }

   public void setDesiredInitialState(FramePoint3D initialCoMLocation, FramePoint3D initialCoPLocation, FrameVector3D initialCoMVelocity)
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

   public void setDesiredFinalState(FramePoint3D finalCoMLocation, FramePoint3D finalCoPLocation, FrameVector3D finalCoMVelocity)
   {
      this.desiredFinalCoMPosition.setIncludingFrame(finalCoMLocation);
      this.desiredFinalCoMVelocity.setIncludingFrame(finalCoMVelocity);
      this.desiredFinalCoPPosition.setIncludingFrame(finalCoPLocation);
   }

   private void generateAccelerationMinimizationObjective()
   {
      int numberOfDecisionVariables = numberOfSegments.getIntegerValue() * (comAxis.length * numberOfCoMTrajectoryCoefficients.getIntegerValue()
            + copAxis.length * numberOfCoPTrajectoryCoefficients.getIntegerValue() + numberOfScalarTrajectoryCoefficients.getIntegerValue());
      solver_objH.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      solver_objf.reshape(numberOfDecisionVariables, 1);
      solver_objH.zero();
      solver_objf.zero();
      List<Trajectory3D> comTrajectories = sqpSolution.comTrajectories;
      for (int i = 0; i < segmentList.size(); i++)
      {
         double segmentDuration = segmentList.get(i).getSegmentDuration();
         Trajectory3D comTrajectory = comTrajectories.get(i);
         for (Axis axis : comAxis)
         {
            Trajectory axisCoMTrajectory = comTrajectory.getTrajectory(axis);
            axisCoMTrajectory.getCoefficientVector(tempA1);
            constraintGenerationHelper.generateAccelerationMinimizationObjective(tempJ1, tempC1, tempA1,
                                                                                 numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1, segmentDuration);
            addIntraSegmentObjective(i, axis, tempJ1, tempC1);
         }
      }
   }

   private void addIntraSegmentObjective(int segmentIndex, Axis axis, DenseMatrix64F H, DenseMatrix64F f)
   {
      int indexToInsertAt = segmentIndex * (comAxis.length * numberOfCoMTrajectoryCoefficients.getIntegerValue()
            + copAxis.length * numberOfCoPTrajectoryCoefficients.getIntegerValue() + numberOfScalarTrajectoryCoefficients.getIntegerValue())
            + axis.ordinal() * numberOfCoMTrajectoryCoefficients.getIntegerValue();
      CommonOps.insert(H, solver_objH, indexToInsertAt, indexToInsertAt);
      CommonOps.insert(f, solver_objf, indexToInsertAt, 0);
   }

   public void submitSegmentList(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      this.segmentList = segmentList;
   }

   private final DenseMatrix64F tempJ1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempC1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempA1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempA2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempJ2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempC2 = new DenseMatrix64F(0, 1);

   public void updateSolution()
   {
      int nofCoMCoeffs = numberOfCoMTrajectoryCoefficients.getIntegerValue();
      int nofCoPCoeffs = numberOfCoPTrajectoryCoefficients.getIntegerValue();
      int nofScalarCoeffs = numberOfScalarTrajectoryCoefficients.getIntegerValue();
      int colsPerSegment = 3 * nofCoMCoeffs + 2 * nofCoPCoeffs + 1 * nofScalarCoeffs;
      List<Trajectory3D> comTrajectories = sqpSolution.comTrajectories;
      List<Trajectory3D> copTrajectories = sqpSolution.copTrajectories;
      List<Trajectory> scalarTrajectories = sqpSolution.scalarProfile;
      for (int i = 0; i < segmentList.size(); i++)
      {
         Trajectory3D comTrajectory = comTrajectories.get(i);
         Trajectory3D copTrajectory = copTrajectories.get(i);
         Trajectory scalarTrajectory = scalarTrajectories.get(i);
         int segmentIndex = i * colsPerSegment;
         // Update CoM solution
         tempJ1.reshape(nofCoMCoeffs, 1);
         for (Axis axis : comAxis)
         {
            Trajectory axisTrajectory = comTrajectory.getTrajectory(axis);
            int axisIndex = segmentIndex + axis.ordinal() * nofCoMCoeffs;
            CommonOps.extract(solver_qpSoln, axisIndex, axisIndex + nofCoMCoeffs, 0, 1, tempJ1, 0, 0);
            for (int j = 0; j < nofCoMCoeffs; j++)
               axisTrajectory.setDirectly(j, axisTrajectory.getCoefficient(j) + tempJ1.get(j, 0));
         }
         // Update CoP solution
         tempJ1.reshape(nofCoPCoeffs, 1);
         for (Axis axis : copAxis)
         {
            Trajectory axisTrajectory = copTrajectory.getTrajectory(axis);
            int axisIndex = segmentIndex + 3 * nofCoMCoeffs + axis.ordinal() * nofCoPCoeffs;
            CommonOps.extract(solver_qpSoln, axisIndex, axisIndex + nofCoPCoeffs, 0, 1, tempJ1, 0, 0);
            for (int j = 0; j < nofCoPCoeffs; j++)
               axisTrajectory.setDirectly(j, axisTrajectory.getCoefficient(j) + tempJ1.get(j, 0));

         }
         // Update scalar solution         
         tempJ1.reshape(nofScalarCoeffs, 1);
         int axisIndex = segmentIndex + 3 * nofCoMCoeffs + 2 * nofCoPCoeffs;
         CommonOps.extract(solver_qpSoln, axisIndex, axisIndex + nofScalarCoeffs, 0, 1, tempJ1, 0, 0);
         for (int j = 0; j < nofScalarCoeffs; j++)
            scalarTrajectory.setDirectly(j, scalarTrajectory.getCoefficient(j) + tempJ1.get(j, 0));
      }
   }

   public boolean compute()
   {
      reshapeMatrices();
      generateCoMSmoothnessConstraints();
      generateCoPSmoothnessConstraints();
      generateScalarSmoothnessConstraints();
      //generateCoPLocationConstraintsFromContactStates();
      //generateCoMLocationConstraintsFromContactStates();
      generateScalarConstraintsFromContactStates();
      generateInitialFinalCoMLocationConstraintsFromDesireds();
      generateInitialFinalCoPLocationConstraintsFromDesireds();
      generateInitialFinalScalarConstraintsFromDesireds();
      generateDynamicsConstraintsForSegments();
      generateAccelerationMinimizationObjective();
      return submitQPMatricesAndRunOptimization();
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
         List<Double> nodeTimes = generateNodeTimesForConstraints(segmentList.get(i).getSegmentDuration(),
                                                                  numberOfScalarConstraintsPerSegment.getIntegerValue(), false, false);
         scalarTrajectories.get(i).getCoefficientVector(tempA1);
         constraintGenerationHelper.generateScalarConstraintMatrix(tempJ1, tempC1, tempA1, numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1,
                                                                   nodeTimes);
         CommonOps.scale(-1.0, tempJ1);
         if (contactState.getContactType().isRobotSupported())
            inequalityConstraintHandler.addIntraSegmentScalarConstraint(i, tempJ1, tempC1);
         else
            equalityConstraintHandler.addIntraSegmentScalarConstraint(i, tempJ1, tempC1);
      }
   }

   private void generateCoMLocationConstraintsFromContactStates()
   {
      List<Trajectory3D> comTrajectories = sqpSolution.comTrajectories;
      for (int i = 0; i < segmentList.size(); i++)
      {
         ContactState contactState = segmentList.get(i).getContactState();
         if (contactState.getContactType().isRobotSupported())
         {
            Trajectory3D comTrajectory = comTrajectories.get(i);
            contactState.getSupportPolygon(tempPolygon);
            polygonScaler.scaleConvexPolygon(tempPolygon, -comSupportPolygonXYConstraintOffset.getDoubleValue(), tempPolygonForScaling);
            List<Double> nodeTimes = generateNodeTimesForConstraints(segmentList.get(i).getSegmentDuration(),
                                                                     numberOfCoMPositionConstraintsPerSegment.getIntegerValue(), true, true);
            Trajectory xTrajectory = comTrajectory.getTrajectoryX();
            Trajectory yTrajectory = comTrajectory.getTrajectoryY();
            Trajectory zTrajectory = comTrajectory.getTrajectoryZ();
            xTrajectory.getCoefficientVector(tempA1);
            yTrajectory.getCoefficientVector(tempA2);
            constraintGenerationHelper.generateSupportPolygonConstraint(tempJ1, tempJ2, tempC1, tempA1, tempA2, tempPolygonForScaling, nodeTimes,
                                                                        numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1);
            inequalityConstraintHandler.addIntraSegmentMultiAxisCoMXYConstraint(i, tempJ1, tempJ2, tempC1);
            zTrajectory.getCoefficientVector(tempA1);
            constraintGenerationHelper.generateZAxisUpperLowerLimitConstraint(tempJ1, tempC1, tempA1, comZMaxHeightConstraint.getDoubleValue(),
                                                                              comZMinHeightConstraint.getDoubleValue(), nodeTimes,
                                                                              numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1);
            inequalityConstraintHandler.addIntraSegmentCoMConstraint(Axis.Z, i, tempJ1, tempC1);
         }
      }
   }

   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D tempPolygonForScaling = new ConvexPolygon2D();

   private void generateCoPLocationConstraintsFromContactStates()
   {
      List<Trajectory3D> copTrajectories = sqpSolution.copTrajectories;
      for (int i = 0; i < segmentList.size(); i++)
      {
         CollinearForceMotionPlannerSegment segment = segmentList.get(i);
         ContactState contactState = segment.getContactState();
         if (contactState.getContactType().isRobotSupported())
         {
            Trajectory3D copTrajectory = copTrajectories.get(i);
            contactState.getSupportPolygon(tempPolygon);
            List<Double> nodeTimes = generateNodeTimesForConstraints(segment.getSegmentDuration(),
                                                                     numberOfSupportPolygonConstraintsPerSegment.getIntegerValue(), true, true);
            Trajectory xAxisCoPTrajectory = copTrajectory.getTrajectoryX();
            Trajectory yAxisCoPTrajectory = copTrajectory.getTrajectoryY();
            xAxisCoPTrajectory.getCoefficientVector(tempA1);
            yAxisCoPTrajectory.getCoefficientVector(tempA2);
            constraintGenerationHelper.generateSupportPolygonConstraint(tempJ1, tempJ2, tempC1, tempA1, tempA2, tempPolygon, nodeTimes,
                                                                        numberOfCoPTrajectoryCoefficients.getIntegerValue() - 1);
            inequalityConstraintHandler.addIntraSegmentMultiAxisCoPConstraint(i, tempJ1, tempJ2, tempC1);
         }
      }
   }

   private List<Double> generateNodeTimesForConstraints(double segmentDuration, int numberOfConstraints, boolean includeStartTime, boolean includeEndTime)
   {
      tempDoubleList.clear();
      int divisor = (includeStartTime && includeEndTime) ? numberOfConstraints - 1
            : includeEndTime || includeStartTime ? numberOfConstraints : numberOfConstraints + 1;
      double dt = segmentDuration / divisor;
      for (int i = includeStartTime ? 0 : 1; i < divisor; i++)
         tempDoubleList.add(dt * i);
      if (includeEndTime)
         tempDoubleList.add(dt * divisor);
      return tempDoubleList;
   }

   public void generateScalarSmoothnessConstraints()
   {
      List<Trajectory> scalarTrajectories = sqpSolution.scalarProfile;
      int scalarTrajectoryOrder = this.numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1;
      for (int i = 0; i < numberOfSegments.getIntegerValue() - 1; i++)
      {
         double segmentDuration = segmentList.get(i).getSegmentDuration();
         Trajectory segment = scalarTrajectories.get(i);
         segment.getCoefficientVector(tempA1);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, scalarTrajectoryOrder,
                                                                                maxDegreeOfScalarSmoothnessConstraints.getIntegerValue(), segmentDuration);
         Trajectory nextSegment = scalarTrajectories.get(i + 1);
         nextSegment.getCoefficientVector(tempA1);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC2, tempA1, scalarTrajectoryOrder,
                                                                                maxDegreeOfScalarSmoothnessConstraints.getIntegerValue(), 0.0);
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
            axisSegmentTrajectory.getCoefficientVector(tempA1);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, copTrajectoryOrder,
                                                                                   maxDegreeOfCoPSmoothnessConstraints.getIntegerValue(), segmentDuration);
            Trajectory axisNextSegmentTrajectory = nextSegment.getTrajectory(axis);
            axisNextSegmentTrajectory.getCoefficientVector(tempA1);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC2, tempA1, copTrajectoryOrder,
                                                                                   maxDegreeOfCoPSmoothnessConstraints.getIntegerValue(), 0.0);
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
            axisSegmentTrajectory.getCoefficientVector(tempA1);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, comTrajectoryOrder,
                                                                                   maxDegreeOfCoMSmoothnessConstraints.getIntegerValue(), segmentDuration);
            Trajectory axisNextSegmentTrajectory = nextSegment.getTrajectory(axis);
            axisNextSegmentTrajectory.getCoefficientVector(tempA1);
            constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC2, tempA1, comTrajectoryOrder,
                                                                                   maxDegreeOfCoMSmoothnessConstraints.getIntegerValue(), 0.0);
            //PrintTools.debug("Segment " + i + axis.toString() + " " + tempA1.toString() + " " + tempA2.toString());
            //PrintTools.debug("Segment " + i + " " + tempJ1.toString() + " " + tempJ2.toString());
            //PrintTools.debug("Segment " + i + " " + tempC1.toString() + " " + tempC2.toString());
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
      firstScalarSegment.getCoefficientVector(tempA1);
      constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, firstScalarSegment.getNumberOfCoefficients() - 1, 0, 0.0);
      CommonOps.subtract(initialOmega, tempC1, tempC1);
      equalityConstraintHandler.addIntraSegmentScalarConstraint(0, tempJ1, tempC1);

      double finalOmega = -gravity.getZ() / desiredFinalCoMPosition.getZ();
      int lastSegmentIndex = numberOfSegments.getIntegerValue() - 1;
      Trajectory lastScalarSegment = sqpSolution.scalarProfile.get(lastSegmentIndex);
      lastScalarSegment.getCoefficientVector(tempA1);
      constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, lastScalarSegment.getNumberOfCoefficients() - 1, 0,
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
         firstAxisSegment.getCoefficientVector(tempA1);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, firstAxisSegment.getNumberOfCoefficients() - 1, 0, 0.0);
         CommonOps.subtract(desiredInitalValue, tempC1, tempC1);
         equalityConstraintHandler.addIntraSegmentCoPConstraint(axis, 0, tempJ1, tempC1);

         double desiredFinalValue = desiredFinalCoPPosition.getElement(axis.ordinal());
         Trajectory lastAxisSegment = lastCoPSegment.getTrajectory(axis);
         lastAxisSegment.getCoefficientVector(tempA1);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, lastAxisSegment.getNumberOfCoefficients() - 1, 0,
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
         firstAxisSegment.getCoefficientVector(tempA1);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ1, tempC1, tempA1, firstAxisSegment.getNumberOfCoefficients() - 1, 1, 0.0);
         CommonOps.subtractEquals(tempC2, tempC1);
         equalityConstraintHandler.addIntraSegmentCoMConstraint(axis, 0, tempJ1, tempC2);

         tempC2.set(0, 0, desiredFinalCoMPosition.getElement(axis.ordinal()));
         tempC2.set(1, 0, desiredFinalCoMVelocity.getElement(axis.ordinal()));
         Trajectory lastAxisSegment = lastCoMSegment.getTrajectory(axis);
         lastAxisSegment.getCoefficientVector(tempA1);
         constraintGenerationHelper.generateDerivativeCoefficientsAndBiasMatrix(tempJ2, tempC1, tempA1, lastAxisSegment.getNumberOfCoefficients() - 1, 1,
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
         List<Double> nodeTimesList = generateNodeTimesForConstraints(segmentList.get(i).getSegmentDuration(),
                                                                      numberOfDynamicsConstraintsPerSegment.getIntegerValue(), false, true);
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
                                                                              numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1,
                                                                              gravity.getElement(axis.ordinal()));
            //PrintTools.debug("Coeffs: " + comCoefficients.toString() + " " + copCoefficients.toString() + " " + scalarCoefficients.toString());
            //PrintTools.debug("Axis" + axis.toString() + " " + comConstraints.toString() + " " + copConstraints.toString() + " " + scalarConstraints.toString() + " " + constraintViolation.toString());
            equalityConstraintHandler.addIntraSegmentMultiQuantityConstraints(axis, i, comConstraints, copConstraints,
                                                                              scalarConstraints, constraintViolation);
         }
         Trajectory zCoMTrajectory = comTrajectory.getTrajectory(Axis.Z);
         zCoMTrajectory.getCoefficientVector(comCoefficients);
         copCoefficients.reshape(numberOfCoPTrajectoryCoefficients.getIntegerValue(), 1);
         copCoefficients.zero();
         scalarTrajectory.getCoefficientVector(scalarCoefficients);
         constraintGenerationHelper.generateDynamicsCollocationConstraints(comConstraints, copConstraints, scalarConstraints, constraintViolation,
                                                                           nodeTimesList, comCoefficients, copCoefficients, scalarCoefficients,
                                                                           numberOfCoMTrajectoryCoefficients.getIntegerValue() - 1,
                                                                           numberOfCoPTrajectoryCoefficients.getIntegerValue() - 1,
                                                                           numberOfScalarTrajectoryCoefficients.getIntegerValue() - 1,
                                                                           gravity.getElement(Axis.Z.ordinal()));
         equalityConstraintHandler.addIntraSegmentMultiQuantityConstraintsForZAxis(i, comConstraints, scalarConstraints,
                                                                                   constraintViolation);
      }
   }

   private final DenseMatrix64F regularization = new DenseMatrix64F(0, 1);

   private boolean submitQPMatricesAndRunOptimization()
   {
      int numberOfDecisionVariables = solver_objH.numCols;
      solver_conLb.reshape(numberOfDecisionVariables, 1);
      solver_conUb.reshape(numberOfDecisionVariables, 1);
      for (int i = 0; i < numberOfDecisionVariables; i++)
      {
         solver_conLb.set(i, 0, -Double.MAX_VALUE);
         solver_conUb.set(i, 0, Double.MAX_VALUE);
      }
      double norm = 0.0;
      double weird = 0.0;
      for (int i = 0; i < solver_objH.numRows; i++)
      {
         for (int j = 0; j < solver_objH.numCols; j++)
         {
            norm += Math.abs(solver_objH.get(i, j));
            weird += solver_objH.get(i, j);
         }
      }
      PrintTools.debug(norm + " " + weird + " " + numberOfDecisionVariables);
      regularization.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      CommonOps.setIdentity(regularization);
      CommonOps.scale(1.0e-7, regularization);
      CommonOps.addEquals(solver_objH, regularization);

      qpSolver.setQuadraticCostFunction(solver_objH, solver_objf, 0.0);
      qpSolver.setLinearEqualityConstraints(equalityConstraintHandler.getCoefficientMatrix(), equalityConstraintHandler.getBiasMatrix());
      qpSolver.setLinearInequalityConstraints(inequalityConstraintHandler.getCoefficientMatrix(), inequalityConstraintHandler.getBiasMatrix());
      qpSolver.setUpperBounds(solver_conUb);
      qpSolver.setLowerBounds(solver_conLb);

      try
      {
         qpSolver.solve(solver_qpSoln);
      }
      catch (NoConvergenceException e)
      {
         errorCode.set(-1);
         return false;
      }
      boolean doesSolnContainNaN = doesSolutionContainNaN();
      if (doesSolnContainNaN)
      {
         errorCode.set(-2);
      }
      PrintTools.debug("Solver has finished running");
      return !doesSolnContainNaN;
   }

   private boolean doesSolutionContainNaN()
   {
      if (solver_qpSoln.numCols != 1)
         return true;
      for (int i = 0; i < solver_qpSoln.numRows; i++)
         if (!Double.isFinite(solver_qpSoln.get(i, 0)))
            return true;
      return false;
   }
}