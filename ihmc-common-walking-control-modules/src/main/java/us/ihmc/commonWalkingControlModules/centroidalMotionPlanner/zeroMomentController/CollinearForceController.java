package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedCoMMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.MotionPlannerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.TransformHelperTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class CollinearForceController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum SupportStateEnum
   {
      SUPPORTED, NOT_SUPPORTED;
   };

   private final YoVariableRegistry registry = new YoVariableRegistry("CollinearForceController");
   private final LinearMotionController linearController;
   private final CentroidalMotionPlanGenerator motionPlanner;
   private final CentroidalMotionPlan motionPlan;
   private final List<ContactState> contactStatePlanForPlanner = new ArrayList<>();
   private final RecyclingArrayList<ContactState> contactStatePlan = new RecyclingArrayList<>(ContactState.class);
   private final YoDouble stateStartTime;
   private final YoDouble controllerTime;
   private final YoDouble timeInState;
   private final YoDouble currentStateDuration;
   private final YoDouble remainingTimeInState;
   private final YoEnum<SupportStateEnum> estimatedState;
   private final YoEnum<SupportStateEnum> plannedState;
   private final YoEnum<SupportStateEnum> controlState;
   private final YoDouble nominalHeight;
   private final ContactState terminalStateForPlanner = new ContactState();

   private final YoFramePoint finalCoMPositionForPlanner;
   private final YoFrameVector finalCoMVelocityForPlanner;
   private final YoFramePoint finalCoPPositionForPlanner;

   private final YoFrameVector desiredLinearMomentumRateOfChangeCommand;
   private final CentroidalStateReadOnly estimatedCoMState;
   private final YoFramePoint computedCoP;
   private final YoFrameVector computedGroundReactionForce;
   private final YoFrameVector gravity;
   private final YoDouble mass;
   private ExternalTransitionTrigger externalTransitionTrigger = null;
   // Temp variables 
   private final FrameVector3D tempFrameVector = new FrameVector3D();
   private final FramePoint3D tempFramePoint = new FramePoint3D();

   public CollinearForceController(YoDouble yoTime, CentroidalStateReadOnly centroidalState, YoVariableRegistry parentRegistry)
   {
      String namePrefix = "CollinearController";
      linearController = new LinearMotionController(registry);
      motionPlanner = new CollinearForceBasedCoMMotionPlanner(registry);
      motionPlan = motionPlanner.getMotionPlanReference();

      // State stuff 
      controllerTime = yoTime;
      stateStartTime = new YoDouble(namePrefix + "StateStartTime", registry);
      timeInState = new YoDouble(namePrefix + "TimeInState", parentRegistry);
      currentStateDuration = new YoDouble(namePrefix + "PlannedStateDuration", registry);
      remainingTimeInState = new YoDouble(namePrefix + "RemainingTimeInState", registry);
      estimatedState = new YoEnum<>(namePrefix + "EstimatedSupportState", registry, SupportStateEnum.class);
      plannedState = new YoEnum<>(namePrefix + "PlannedSupportState", registry, SupportStateEnum.class);
      controlState = new YoEnum<>(namePrefix + "SupportStateForControl", registry, SupportStateEnum.class);

      nominalHeight = new YoDouble(namePrefix + "NominalCoMHeight", registry);
      gravity = new YoFrameVector(namePrefix + "Gravity", worldFrame, registry);
      mass = new YoDouble(namePrefix + "Mass", registry);
      // Control parameters
      this.estimatedCoMState = centroidalState;

      desiredLinearMomentumRateOfChangeCommand = new YoFrameVector(namePrefix + "LinearMomentumRateOfChangeCommand", worldFrame, registry);
      computedCoP = new YoFramePoint(namePrefix + "ComputedCenterOfPressure", worldFrame, registry);
      computedGroundReactionForce = new YoFrameVector(namePrefix + "ComputedGroundReactionForce", worldFrame, registry);
      finalCoMPositionForPlanner = new YoFramePoint(namePrefix + "FinalCoMPositionForPlanner", worldFrame, registry);
      finalCoMVelocityForPlanner = new YoFrameVector(namePrefix + "FinalCoMVelocityForPlanner", worldFrame, registry);
      finalCoPPositionForPlanner = new YoFramePoint(namePrefix + "FinalCoPPositionForPlanner", worldFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void intialize(MotionPlannerParameters plannerParameters, FrameVector3DReadOnly linearMomentumProportionalGains,
                         FrameVector3DReadOnly linearMomentumDerivativeGains, FrameVector3DReadOnly gravity, double mass, double nominalHeight)
   {
      this.nominalHeight.set(nominalHeight);
      this.mass.set(mass);
      this.gravity.set(gravity);
      motionPlanner.initialize(plannerParameters, gravity);
      linearController.setLinearMomentumFeedbackGains(linearMomentumProportionalGains, linearMomentumDerivativeGains);
   }

   public void appendContactStateList(List<ContactState> contactStatesToAppend)
   {
      for (int i = 0; i < contactStatesToAppend.size(); i++)
         appendContactState(contactStatesToAppend.get(i));
   }

   public void appendContactState(ContactState contactStateToAppend)
   {
      contactStatePlan.add().set(contactStateToAppend);
   }

   private void setupPlanner()
   {
      boolean clipPlan = motionPlanner.getMaximumNumberOfContactStatesToPlan() < contactStatePlan.size();
      int numberOfContactStatesToPlan = clipPlan ? motionPlanner.getMaximumNumberOfContactStatesToPlan() : contactStatePlan.size();
      contactStatePlanForPlanner.clear();
      int contactStateIndex = 0;
      for (; contactStateIndex < numberOfContactStatesToPlan; contactStateIndex++)
         contactStatePlanForPlanner.add(contactStatePlan.get(contactStateIndex));
      // TODO add fake state as needed to ease planners burden 
      if (clipPlan)
         generateFinalStateDesired(contactStatePlan.get(contactStateIndex - 1), null);
      else
         generateFinalStateDesired(contactStatePlan.get(contactStateIndex - 1), contactStatePlan.get(contactStateIndex));
   }

   public void doControl()
   {
      updateEstimates();
      if (checkTransitionConditions())
         transitionToNextState();
      updateDesireds();
      updateControlCommand();
   }

   public void setExternalStateChangeModule(ExternalTransitionTrigger externalTrigger)
   {
      this.externalTransitionTrigger = externalTrigger;
   }

   private boolean checkTransitionConditions()
   {
      boolean transition;
      if (externalTransitionTrigger != null)
         transition = externalTransitionTrigger.checkTransitionConditions();
      else
         transition = readyToTransition();
      return transition;
   }

   public boolean readyToTransition()
   {
      return remainingTimeInState.getDoubleValue() <= 1e-10;
   }

   private void transitionToNextState()
   {
      if (contactStatePlan.size() > 1)
         contactStatePlan.remove(0);
      ContactState transitioningToState = contactStatePlan.getFirst();
      runPlanner();
      // Update the timing variables
      controlState.set(transitioningToState.isSupported() ? SupportStateEnum.SUPPORTED : SupportStateEnum.NOT_SUPPORTED);
      currentStateDuration.set(transitioningToState.getDuration());
      timeInState.set(0.0);
      remainingTimeInState.set(transitioningToState.getDuration());
      stateStartTime.set(controllerTime.getDoubleValue());
   }

   private void updateDesireds()
   {
      motionPlan.compute(timeInState.getDoubleValue());
      plannedState.set(motionPlan.getPlannedGroundReactionForce().getZ() <= 1e-10 ? SupportStateEnum.NOT_SUPPORTED : SupportStateEnum.SUPPORTED);

      linearController.setPlannedCoM(motionPlan.getPlannedCoMPosition(), motionPlan.getPlannedCoMVelocity());
      linearController.setFeedforwardLinearAcceleration(motionPlan.getPlannedCoMAcceleration());
      linearController.doControl();
   }

   private void updateEstimates()
   {
      // It is assumed that the centroidal state is updated
      // Update timing variables
      timeInState.set(controllerTime.getDoubleValue() - stateStartTime.getDoubleValue());
      remainingTimeInState.set(currentStateDuration.getDoubleValue() - timeInState.getDoubleValue());

      // Update indirect control quantitites
      CentroidalModelTools.computeGroundReactionForceFromAcceleration(tempFrameVector, estimatedCoMState.getLinearAcceleration(), gravity,
                                                                      mass.getDoubleValue());
      computedGroundReactionForce.set(tempFrameVector);
      CentroidalModelTools.computeCenterOfPressureForFlatGround(estimatedCoMState.getPosition(), 0.0, computedGroundReactionForce,
                                                                estimatedCoMState.getAngularAcceleration(), tempFramePoint);
      // Update direct control quantities
      linearController.setEstimatedCoM(estimatedCoMState.getPosition(), estimatedCoMState.getLinearVelocity());

      // Update system state
      boolean isVerticalAccelerationSimilarToFreeFall = estimatedCoMState.getLinearAcceleration().getZ() < gravity.getZ() + 5e-2;
      estimatedState.set(isVerticalAccelerationSimilarToFreeFall ? SupportStateEnum.NOT_SUPPORTED : SupportStateEnum.SUPPORTED);
   }

   private void updateControlCommand()
   {
      linearController.doControl();
      desiredLinearMomentumRateOfChangeCommand.set(linearController.getLinearAccelerationCommand());
      desiredLinearMomentumRateOfChangeCommand.scale(mass.getDoubleValue());
   }

   private final Point2D tempPoint2D = new Point2D();
   private final FramePose3D tempPose = new FramePose3D();

   private void generateFinalStateDesired(ContactState lastStateForPlanner, ContactState availableNextState)
   {
      //TODO add some fancy heuristics here to speed up the calculation
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      for (RobotSide side : RobotSide.values)
      {
         if (!lastStateForPlanner.footInContact.get(side))
            continue;
         ConvexPolygon2D footPolygon = lastStateForPlanner.footSupportPolygons.get(side);
         tempPoint2D.set(footPolygon.getCentroid());
         tempPose.setIncludingFrame(lastStateForPlanner.getPose(side));
         tempPose.changeFrame(worldFrame);
         TransformHelperTools.transformFromPoseToReferenceFrameByProjection(tempPose, tempPoint2D);
         x += tempPoint2D.getX();
         y += tempPoint2D.getY();
      }
      x /= 2.0;
      y /= 2.0;
      z = nominalHeight.getDoubleValue();
   }

   private void runPlanner()
   {
      setupPlanner();
      motionPlanner.submitContactStateList(contactStatePlanForPlanner);
      motionPlanner.setInitialState(estimatedCoMState.getPosition(), estimatedCoMState.getLinearVelocity(), computedCoP);
      motionPlanner.setFinalState(finalCoMPositionForPlanner, finalCoMVelocityForPlanner, finalCoPPositionForPlanner);
      // TODO option to set the final state as an objective based on future plan
      if (!motionPlanner.compute())
         throw new RuntimeException("Unable to centroidal motion plan for current contact states");
   }
}
