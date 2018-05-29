package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * <p>Implements a SQP based centroidal motion planning approach to enable a larger variety of motions 
 * This models the robot as a point mass and a contact surface that can exert a restricted force on the
 * point mass. Constraints are imposed on plan generated to ensure that the force computed acts collinear 
 * to the CoP (constrained to be within the contact surface) and CoM. This ensures that the plan generated 
 * does not result in angular momentum changes. </p>
 * Essentially the mathematical equation being solved is <b>xddot = u (x - &n;) + g</b> where <b> x </b> is the CoM location,
 * <i> u  </i> is a scalar, <b>&nu;</b> is the CoP and <b> g </b> is the acceleration due to gravity
 * <p> All motion planning is done in the {@code ReferenceFrame#getWorldFrame()} </p>
 * @author Apoorv S
 *
 */
public class CollinearForceBasedCoMMotionPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final int numberOfScalarTrajectoryCoefficients = 4;
   public static final int numberOfCoMTrajectoryCoefficients = 8;
   public static final int numberOfCoPTrajectoryCoefficients = 8;

   private final YoFramePoint initialCoMPosition;
   private final YoFramePoint initialCoPPosition;
   private final YoFrameVector initialCoMVelocity;
   private final YoFramePoint finalCoMPosition;
   private final YoFramePoint finalCoPPosition;
   private final YoFrameVector finalCoMVelocity;

   private final YoVariableRegistry registry;
   private final YoDouble nominalPlannerSegmentTime;
   private final YoDouble minPlannerSegmentTime;
   private final YoInteger numberOfContactStatesToPlan;
   private final YoInteger maxNumberOfSQPIterations;
   private final YoDouble consolidatedConvergenceThreshold;
   private final YoDouble individualAxisConvergenceThreshold;
   
   private final YoDouble dynamicsViolation;
   private final YoDouble[] axisDynamicsViolation = new YoDouble[Axis.values.length];
   
   private final YoInteger numberOfContactStates;
   private final YoInteger numberOfPlanningSegments;
   private final YoInteger numberOfElapsedSQPIterations;
   private final YoBoolean hasPlanConverged;
   private final YoBoolean hasPlannerFailed;
   private final YoBoolean hasInitialStateBeenSet;
   private final YoBoolean hasFinalStateBeenSet;

   private final RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentList;
   private final RecyclingArrayList<ContactState> contactStateList;

   private final CollinearForceBasedPlannerResult sqpSolution;
   private final CollinearForceBasedPlannerOptimizationControlModule optimizationControlModule;
   private final CollinearForceBasedPlannerSeedSolutionGenerator initialSolutionGenerator;
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public CollinearForceBasedCoMMotionPlanner(YoVariableRegistry parentRegistry)
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);

      initialCoMPosition = new YoFramePoint(namePrefix + "InitialCoMLocation", worldFrame, registry);
      initialCoPPosition = new YoFramePoint(namePrefix + "InitialCoPLocation", worldFrame, registry);
      initialCoMVelocity = new YoFrameVector(namePrefix + "InitialCoMVelocity", worldFrame, registry);
      finalCoMPosition = new YoFramePoint(namePrefix + "FinalCoMLocation", worldFrame, registry);
      finalCoPPosition = new YoFramePoint(namePrefix + "FinalCoPLocation", worldFrame, registry);
      finalCoMVelocity = new YoFrameVector(namePrefix + "FinalCoMVelocity", worldFrame, registry);

      dynamicsViolation = new YoDouble(namePrefix + "CummulativeDynamicsViolation", registry);
      for(Axis axis : Axis.values)
         axisDynamicsViolation[axis.ordinal()] = new YoDouble(namePrefix + axis.toString() + "DynamicsViolation", registry);
      hasPlanConverged = new YoBoolean(namePrefix + "HasPlannerConverged", registry);
      hasPlannerFailed = new YoBoolean(namePrefix + "HasPlannerFailed", registry);
      hasInitialStateBeenSet = new YoBoolean(namePrefix + "HasInitialStateBeenSet", registry);
      hasFinalStateBeenSet = new YoBoolean(namePrefix + "HasFinalStateBeenSet", registry);
      numberOfElapsedSQPIterations = new YoInteger(namePrefix + "NumberOfElapsedSQPIterations", registry);
      numberOfPlanningSegments = new YoInteger(namePrefix + "NumberOfPlanningSegments", registry);
      numberOfContactStates = new YoInteger(namePrefix + "NumberOfContactStates", registry);

      nominalPlannerSegmentTime = new YoDouble(namePrefix + "MaxPlannerSegmentTime", registry);
      minPlannerSegmentTime = new YoDouble(namePrefix + "MinPlannerSegmentTime", registry);
      numberOfContactStatesToPlan = new YoInteger(namePrefix + "NumberOfContactStatesToPlan", registry);
      maxNumberOfSQPIterations = new YoInteger(namePrefix + "MaxNumberOfSQPIterations", registry);
      consolidatedConvergenceThreshold = new YoDouble(namePrefix + "ConsolidatedConvergenceThreshold", registry);
      individualAxisConvergenceThreshold = new YoDouble(namePrefix + "IndividualAxisConvergenceThreshold", registry);

      sqpSolution = new CollinearForceBasedPlannerResult(registry);
      optimizationControlModule = new CollinearForceBasedPlannerOptimizationControlModule(sqpSolution, numberOfPlanningSegments, registry);
      initialSolutionGenerator = new CollinearForceBasedPlannerSeedSolutionGenerator(registry); // TODO set this up
      contactStateList = new RecyclingArrayList<>(100, ContactState.class);
      segmentList = new RecyclingArrayList<>(100, CollinearForceMotionPlannerSegment.class);
      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(CollinearForcePlannerParameters parameters, FrameVector3DReadOnly gravity)
   {
      maxNumberOfSQPIterations.set(parameters.getMaxSQPIterations());
      consolidatedConvergenceThreshold.set(parameters.getConsolidatedConvergenceThreshold());
      individualAxisConvergenceThreshold.set(parameters.getIndividualAxisConvergenceThreshold());
      nominalPlannerSegmentTime.set(parameters.getNominalPlannerSegmentTime());
      minPlannerSegmentTime.set(parameters.getMinPlannerSegmentTime());
      numberOfContactStatesToPlan.set(parameters.getNumberOfContactStatesToPlan());
      
      sqpSolution.initialize(gravity, parameters.getRobotMass());
      initialSolutionGenerator.initialize(sqpSolution, gravity, parameters);
      optimizationControlModule.initialize(parameters, gravity);
   }

   public void reset()
   {
      hasPlanConverged.set(false);
      hasPlannerFailed.set(false);
      numberOfElapsedSQPIterations.set(0);
      numberOfContactStates.set(0);
      numberOfPlanningSegments.set(0);
      contactStateList.clear();
      segmentList.clear();
      optimizationControlModule.reset();
      initialSolutionGenerator.reset();
      clearInitialState();
      clearFinalState();
      sqpSolution.reset();
   }
   
   public void clearInitialState()
   {
      hasInitialStateBeenSet.set(false);
      initialCoMPosition.setToNaN();
      initialCoMVelocity.setToNaN();
      initialCoPPosition.setToNaN();
   }
   
   public void clearFinalState()
   {
      hasFinalStateBeenSet.set(false);
      finalCoMPosition.setToNaN();
      finalCoMVelocity.setToNaN();
      finalCoPPosition.setToNaN();
   }

   public void setInitialState(FramePoint3D initialCoMLocation, FrameVector3D initialCoMVelocity, FramePoint3D initialCoPLocation)
   {
      hasInitialStateBeenSet.set(true);
      tempPoint.setIncludingFrame(initialCoMLocation);
      tempPoint.changeFrame(worldFrame);
      this.initialCoMPosition.set(tempPoint);

      tempPoint.setIncludingFrame(initialCoPLocation);
      tempPoint.changeFrame(worldFrame);
      this.initialCoPPosition.set(tempPoint);

      tempVector.setIncludingFrame(initialCoMVelocity);
      tempVector.changeFrame(worldFrame);
      this.initialCoMVelocity.set(tempVector);
   }

   public void setFinalState(FramePoint3D finalCoMLocation, FrameVector3D finalCoMVelocity, FrameVector3D finalCoPLocation)
   {
      hasFinalStateBeenSet.set(true);
      tempPoint.setIncludingFrame(finalCoMLocation);
      tempPoint.changeFrame(worldFrame);
      this.finalCoMPosition.set(tempPoint);

      tempPoint.setIncludingFrame(finalCoPLocation);
      tempPoint.changeFrame(worldFrame);
      this.finalCoPPosition.set(tempPoint);

      tempVector.setIncludingFrame(finalCoMVelocity);
      tempVector.changeFrame(worldFrame);
      this.finalCoMVelocity.set(tempVector);
   }

   public void processContactStateList()
   {
      segmentList.clear();
      ContactState contactState = contactStateList.get(0);
      int i = 0;
      int numberOfContactStatesToProcess = Math.min(numberOfContactStatesToPlan.getIntegerValue(), numberOfContactStates.getIntegerValue() - 1);
      for (; i < numberOfContactStatesToProcess; i++)
      {
         ContactState nextContactState = contactStateList.get(i + 1);
         double contactStateDuration = contactState.getDuration();
         int numberOfSegmentsInContactState = getNumberOfSegmentsInContactStates(contactStateDuration);
         for (int j = 0; j < numberOfSegmentsInContactState; j++)
         {
            CollinearForceMotionPlannerSegment segment = segmentList.add();
            segment.setContactState(contactState);
            segment.setNextSegmentContactState(contactState);
            segment.setSegmentDuration(nominalPlannerSegmentTime.getDoubleValue());
            segment.setContactStateChangeFlag(false);
         }
         CollinearForceMotionPlannerSegment segment = segmentList.add();
         segment.setContactState(contactState);
         segment.setNextSegmentContactState(nextContactState);
         segment.setSegmentDuration(contactStateDuration - numberOfSegmentsInContactState * nominalPlannerSegmentTime.getDoubleValue());
         segment.setContactStateChangeFlag(true);
         contactState = nextContactState;
      }

      double contactStateDuration = contactState.getDuration();
      int numberOfSegmentsInContactState = getNumberOfSegmentsInContactStates(contactStateDuration) + 1;
      for (int j = 0; j < numberOfSegmentsInContactState; j++)
      {
         CollinearForceMotionPlannerSegment segment = segmentList.add();
         segment.setContactState(contactState);
         segment.setNextSegmentContactState(contactState);
         segment.setSegmentDuration(nominalPlannerSegmentTime.getDoubleValue());
         segment.setContactStateChangeFlag(false);
      }
      PrintTools.debug("Number of segments:" + segmentList.size());
   }

   private int getNumberOfSegmentsInContactStates(double contactStateDuration)
   {
      int quotient = (int) Math.floor(contactStateDuration / nominalPlannerSegmentTime.getDoubleValue());
      double remainder = contactStateDuration - quotient * nominalPlannerSegmentTime.getDoubleValue();
      if (remainder < minPlannerSegmentTime.getDoubleValue())
         return quotient - 1;
      else
         return quotient;
   }

   public void clearContactStateList()
   {
      reset();
   }

   public void appendContactStateToList(ContactState contactStateToAppend)
   {
      if (isNodeValid(contactStateToAppend))
      {
         numberOfContactStates.increment();
         contactStateList.add().set(contactStateToAppend);
      }
   }

   public List<ContactState> getContactStateList()
   {
      return contactStateList;
   }

   private boolean checkIsDynamicsViolationBelowThresholds()
   {
      dynamicsViolation.set(0.0);
      boolean flag = true; 
      flag &= !sqpSolution.didIterationConverge();

      for (Axis axis : Axis.values)
      {
         YoDouble axisViolation = axisDynamicsViolation[axis.ordinal()];
         axisViolation.set(Math.abs(sqpSolution.getViolation(axis)));
         dynamicsViolation.add(axisViolation);
         flag &= axisViolation.getDoubleValue() < individualAxisConvergenceThreshold.getDoubleValue();
      }
      flag &= dynamicsViolation.getDoubleValue() < consolidatedConvergenceThreshold.getDoubleValue();
      return flag;
   }

   private boolean isNodeValid(ContactState contactStateToCheck)
   {
      if (contactStateToCheck.getDuration() <= 0.0f)
         return false;
      if (contactStateToCheck.getReferenceFrame() == null)
         return false;
      if (contactStateToCheck.isSupported() && contactStateToCheck.getNumberOfSupportPolygonVertices() <= 0)
         return false;
      return true;
   }

   public void runIterations(int numberOfSQPIterationsToRun)
   {
      if (isFirstQPRun())
      {
         processContactStateList();
         generateSeedSolution();
         setupOptimizationControlModule();
      }

      for (int i = 0; i < numberOfSQPIterationsToRun; i++)
      {
         if (!hasPlanConverged() && !hasPlannerFailed())
         {
            if (!optimizationControlModule.compute())
            {
               hasPlannerFailed.set(true);
               break;
            }
            optimizationControlModule.updateSolution();
            hasPlanConverged.set(checkIsDynamicsViolationBelowThresholds());
            numberOfElapsedSQPIterations.increment();
            hasPlannerFailed.set(numberOfElapsedSQPIterations.getIntegerValue() > maxNumberOfSQPIterations.getIntegerValue());
         }
         else
            break;
      }
   }

   private void setupOptimizationControlModule()
   {
      if(hasInitialStateBeenSet.getBooleanValue())
         optimizationControlModule.setDesiredInitialState(initialCoMPosition, initialCoPPosition, initialCoMVelocity);
      else
      {
         segmentList.get(0).getContactState().getSupportPolygonCentroid(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.setZ(0.435);
         tempVector.setToZero(worldFrame);
         optimizationControlModule.setDesiredInitialState(tempPoint, tempPoint, tempVector);
      }
      if(hasFinalStateBeenSet.getBooleanValue())
         optimizationControlModule.setDesiredFinalState(initialCoMPosition, finalCoPPosition, finalCoMVelocity);
      else
      {
         segmentList.getLast().getContactState().getSupportPolygonCentroid(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.setZ(0.435);
         tempVector.setToZero(worldFrame);
         optimizationControlModule.setDesiredFinalState(tempPoint, tempPoint, tempVector);
      }
      optimizationControlModule.submitSegmentList(segmentList);
   }

   private void generateSeedSolution()
   {
      initialSolutionGenerator.submitSegmentList(segmentList);
      initialSolutionGenerator.setInitialState(initialCoMPosition, initialCoPPosition, initialCoMVelocity);
      initialSolutionGenerator.setFinalState(finalCoMPosition, finalCoPPosition, finalCoMVelocity);
      initialSolutionGenerator.computeSeedSolution();
   }

   private boolean isFirstQPRun()
   {
      return numberOfElapsedSQPIterations.getIntegerValue() == 0;
   }

   public boolean hasPlanConverged()
   {
      return hasPlanConverged.getBooleanValue();
   }

   public boolean hasPlannerFailed()
   {
      return hasPlannerFailed.getBooleanValue();
   }

   public List<CollinearForceMotionPlannerSegment> getSegmentList()
   {
      return segmentList;
   }

   public List<Trajectory3D> getCoMTrajectory()
   {
      return sqpSolution.comTrajectories;
   }

   public List<Trajectory3D> getCoPTrajectory()
   {
      return sqpSolution.copTrajectories;
   }

   public List<Trajectory> getScalarTrajectory()
   {
      return sqpSolution.scalarProfile;
   }

   public CollinearForceBasedPlannerResult getSQPSolution()
   {
      return sqpSolution;
   }
}
