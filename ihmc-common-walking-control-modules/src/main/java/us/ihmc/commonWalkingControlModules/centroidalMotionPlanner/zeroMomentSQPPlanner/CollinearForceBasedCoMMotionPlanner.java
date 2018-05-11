package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.BipedContactType;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;
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
   public static final int numberOfScalarTrajectoryCoefficients = 4;
   public static final int numberOfCoMTrajectoryCoefficients = 8;
   public static final int numberOfCoPTrajectoryCoefficients = 8;

   private final YoVariableRegistry registry;
   private final YoDouble maxPlannerSegmentTime;
   private final YoDouble minPlannerSegmentTime;
   private final YoInteger numberOfContactStatesToPlan;
   private final YoInteger maxNumberOfSQPIterations;
   private final YoDouble consolidatedConvergenceThreshold;
   private final YoDouble individualAxisConvergenceThreshold;

   private final YoInteger numberOfContactStates;
   private final YoInteger numberOfPlanningSegments;
   private final YoInteger numberOfElapsedSQPIterations;
   private final YoBoolean hasPlanConverged;
   private final YoBoolean hasPlannerFailed;

   private final RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentList;
   private final RecyclingArrayList<ContactState> contactStateList;

   private final CollinearForceBasedPlannerIterationResult sqpSolution;
   private final CollinearForceBasedPlannerOptimizationControlModule optimizationControlModule;
   private final CollinearForceBasedPlannerSeedSolutionGenerator initialSolutionGenerator;

   public CollinearForceBasedCoMMotionPlanner(FrameVector3DReadOnly gravity, YoVariableRegistry parentRegistry)
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);
      hasPlanConverged = new YoBoolean(namePrefix + "HasPlannerConverged", registry);
      hasPlannerFailed = new YoBoolean(namePrefix + "HasPlannerFailed", registry);
      numberOfElapsedSQPIterations = new YoInteger(namePrefix + "NumberOfElapsedSQPIterations", registry);
      numberOfPlanningSegments = new YoInteger(namePrefix + "NumberOfPlanningSegments", registry);
      numberOfContactStates = new YoInteger(namePrefix + "NumberOfContactStates", registry);

      maxPlannerSegmentTime = new YoDouble(namePrefix + "MaxPlannerSegmentTime", registry);
      minPlannerSegmentTime = new YoDouble(namePrefix + "MinPlannerSegmentTime", registry);
      numberOfContactStatesToPlan = new YoInteger(namePrefix + "NumberOfContactStatesToPlan", registry);
      maxNumberOfSQPIterations = new YoInteger(namePrefix + "MaxNumberOfSQPIterations", registry);
      consolidatedConvergenceThreshold = new YoDouble(namePrefix + "ConsolidatedConvergenceThreshold", registry);
      individualAxisConvergenceThreshold = new YoDouble(namePrefix + "IndividualAxisConvergenceThreshold", registry);

      sqpSolution = new CollinearForceBasedPlannerIterationResult(gravity);
      optimizationControlModule = new CollinearForceBasedPlannerOptimizationControlModule(sqpSolution, gravity, registry);
      initialSolutionGenerator = new CollinearForceBasedPlannerSeedSolutionGenerator(); // TODO set this up
      contactStateList = new RecyclingArrayList<>(100, ContactState.class);
      segmentList = new RecyclingArrayList<>(100, CollinearForceMotionPlannerSegment.class);
      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(CollinearForcePlannerParameters parameters)
   {
      maxNumberOfSQPIterations.set(parameters.getMaxSQPIterations());
      consolidatedConvergenceThreshold.set(parameters.getConsolidatedConvergenceThreshold());
      individualAxisConvergenceThreshold.set(parameters.getIndividualAxisConvergenceThreshold());
      maxPlannerSegmentTime.set(parameters.getMaxPlannerSegmentTime());
      minPlannerSegmentTime.set(parameters.getMinPlannerSegmentTime());
      numberOfContactStatesToPlan.set(parameters.getNumberOfContactStatesToPlan());
      optimizationControlModule.initialize(parameters);
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
   }

   public void setInitialState(FramePoint3D initialCoMLocation, FrameVector3D initialCoMVelocity, FramePoint3D initialCoPLocation)
   {
      optimizationControlModule.setInitialState(initialCoMLocation, initialCoMVelocity, initialCoPLocation);
   }

   public void setFinalState(FramePoint3D finalCoMLocation, FrameVector3D finalCoMVelocity, FrameVector3D finalCoPLocation)
   {
      optimizationControlModule.setFinalState(finalCoMLocation, finalCoMVelocity, finalCoPLocation);
   }

   private void processContactStateList()
   {
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
            segment.setSegmentDuration(maxPlannerSegmentTime.getDoubleValue());
            segment.setContactStateChangeFlag(false);
         }
         CollinearForceMotionPlannerSegment segment = segmentList.add();
         segment.setContactState(contactState);
         segment.setNextSegmentContactState(nextContactState);
         segment.setSegmentDuration(contactStateDuration - numberOfSegmentsInContactState * maxPlannerSegmentTime.getDoubleValue());
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
         segment.setSegmentDuration(maxPlannerSegmentTime.getDoubleValue());
         segment.setContactStateChangeFlag(false);
      }
   }

   private int getNumberOfSegmentsInContactStates(double contactStateDuration)
   {
      int quotient = (int) Math.floor(contactStateDuration / maxPlannerSegmentTime.getDoubleValue());
      double remainder = contactStateDuration - quotient * minPlannerSegmentTime.getDoubleValue();
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

   private boolean checkIsDynamicsViolationIsBelowThresholds()
   {
      double totalDynamicsViolation = 0.0;
      if (!sqpSolution.didIterationConverge())
         return false;
      for (Axis axis : Axis.values)
      {
         double axisViolation = Math.abs(sqpSolution.getViolation(axis));
         totalDynamicsViolation += axisViolation;
         if (axisViolation > individualAxisConvergenceThreshold.getDoubleValue())
            return false;
      }
      if (totalDynamicsViolation > consolidatedConvergenceThreshold.getDoubleValue())
         return false;
      return true;
   }

   private boolean isNodeValid(ContactState contactStateToCheck)
   {
      if (contactStateToCheck.getDuration() <= 0.0f)
         return false;
      if (contactStateToCheck.getContactType() == null)
         return false;
      if (contactStateToCheck.getReferenceFrame() == null)
         return false;
      if (contactStateToCheck.getContactType().isRobotSupported() && contactStateToCheck.getNumberOfSupportPolygonVertices() <= 0)
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
            optimizationControlModule.generateLinearizedSystemObjective();
            optimizationControlModule.generateLinearizedSystemConstraints();
            if (!optimizationControlModule.compute())
            {
               hasPlannerFailed.set(true);
               break;
            }
            optimizationControlModule.updateSolution();
            hasPlanConverged.set(checkIsDynamicsViolationIsBelowThresholds());
            numberOfElapsedSQPIterations.increment();
            hasPlannerFailed.set(numberOfElapsedSQPIterations.getIntegerValue() > maxNumberOfSQPIterations.getIntegerValue());
         }
         else
            break;
      }
   }

   private void setupOptimizationControlModule()
   {
      // TODO Auto-generated method stub

   }

   private void generateSeedSolution()
   {
      
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
}
