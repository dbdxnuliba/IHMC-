package us.ihmc.avatar.networkProcessor.continuousPlanningToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class BipedContinuousPlanningToolboxController extends ToolboxController
{
   private static final boolean debug = false;
   private static final boolean verbose = true;

   private static final double defaultTransferDuration = 3.0;
   private static final double defaultSwingDuration = 5.0;

   private static final RobotSide defaultInitialSide = RobotSide.LEFT;

   private static final double proximityGoal = 1.0e-2;
   private static final int defaultStartPlanId = 13;

   private static final int defaultNumberOfStepsToLeaveUnchanged = 3;

   private final AtomicReference<FootstepPlanningToolboxOutputStatus> latestPlannerOutput = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> currentPlannerOutput = new AtomicReference<>();
   private final AtomicReference<BipedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<List<FootstepStatusMessage>> footstepStatusBuffer = new AtomicReference<>(new ArrayList<>());
   private final AtomicReference<RobotSide> expectedInitialSteppingSide = new AtomicReference<>();

   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();

   private final SideDependentList<Pose3DReadOnly> currentFeetPositions = new SideDependentList<>();
   private final List<FootstepDataMessage> fixedStepQueue = new ArrayList<>();
   private final List<FootstepDataMessage> stepQueue = new ArrayList<>();

   private final YoBoolean isInitialSegmentOfPlan = new YoBoolean("isInitialSegmentOfPlan", registry);
   private final YoInteger numberOfStepsToLeaveUnchanged = new YoInteger("numberOfStepsToLeaveUnchanged", registry);
   private final YoBoolean hasReachedGoal = new YoBoolean("hasReachedGoal", registry);
   private final YoBoolean planningFailed = new YoBoolean("planningFailed", registry);
   private final YoBoolean receivedPlanForLastRequest = new YoBoolean("receivedPlanForLastRequest", registry);
   private final YoBoolean waitingForPlan = new YoBoolean("waitingForPlan", registry);

   private final YoInteger broadcastPlanId = new YoInteger("broadcastPlanId", registry);
   private final YoInteger receivedPlanId = new YoInteger("receivedPlanId", registry);

   private final YoDouble transferDuration = new YoDouble("transferDuration", registry);
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);

   private final IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;
   private final IHMCRealtimeROS2Publisher<ToolboxStateMessage> plannerStatePublisher;

   public BipedContinuousPlanningToolboxController(StatusMessageOutputManager statusOutputManager,
                                                   IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher,
                                                   IHMCRealtimeROS2Publisher<ToolboxStateMessage> plannerStatePublisher,
                                                   YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.planningRequestPublisher = planningRequestPublisher;
      this.plannerStatePublisher = plannerStatePublisher;

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);

      numberOfStepsToLeaveUnchanged.set(defaultNumberOfStepsToLeaveUnchanged);

      transferDuration.set(defaultTransferDuration);
      swingDuration.set(defaultSwingDuration);
   }

   public void processFootstepPlannerOutput(FootstepPlanningToolboxOutputStatus message)
   {
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(message.getFootstepPlanningResult());
      if (!result.validForExecution())
      {
         LogTools.info(("Planner result failed. Terminating because of " + result));
         resetForNewPlan();
         planningFailed.set(true);

         return;
      }
      latestPlannerOutput.set(message);
      receivedPlanId.set(message.getPlanId());
      waitingForPlan.set(false);
   }

   public void processContinuousPlanningRequest(BipedContinuousPlanningRequestPacket message)
   {
      resetForNewPlan();
      planningRequestPacket.set(message);
   }

   public void processFootstepStatusMessage(FootstepStatusMessage message)
   {
      List<FootstepStatusMessage> messages = footstepStatusBuffer.get();
      messages.add(message);
      footstepStatusBuffer.set(messages);
      if (FootstepStatus.fromByte(message.getFootstepStatus()) == FootstepStatus.COMPLETED)
      {
         Pose3D footPose = new Pose3D();
         footPose.setOrientation(message.getActualFootOrientationInWorld());
         footPose.setPosition(message.getActualFootPositionInWorld());
         currentFeetPositions.put(RobotSide.fromByte(message.getRobotSide()), footPose);
      }
   }

   public void processPlanarRegionListMessage(PlanarRegionsListMessage message)
   {
      latestPlanarRegions.set(message);
   }

   @Override
   public boolean initialize()
   {
      resetForNewPlan();

      computeAndSendLatestPlanningRequest();

      return true;
   }

   @Override
   public void updateInternal()
   {
      if (!clearCompletedSteps())
         return;

      checkIfLastPlanFulfilledRequest();

      if (!waitingForPlan.getBooleanValue() && !hasReachedGoal.getBooleanValue() && fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getValue())
      {
         if (!updateStepQueueFromNewPlan())
            return;

         if (!updateFixedStepQueue())
            return;

         updateHasReachedGoal();

         if (!hasReachedGoal.getBooleanValue())
         {
            broadcastLatestPlan();

            computeAndSendLatestPlanningRequest();
         }
      }
   }

   @Override
   public boolean isDone()
   {
      return hasReachedGoal.getBooleanValue() || planningFailed.getBooleanValue();
   }

   private void resetForNewPlan()
   {
      waitingForPlan.set(true);
      isInitialSegmentOfPlan.set(true);

      planningFailed.set(false);
      hasReachedGoal.set(false);
      // TODO don't clear the fixed step queue.
      fixedStepQueue.clear();
      currentFeetPositions.clear();
      stepQueue.clear();
      footstepStatusBuffer.set(new ArrayList<>());
      latestPlannerOutput.set(null);
      expectedInitialSteppingSide.set(defaultInitialSide);

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);
   }

   private void computeAndSendLatestPlanningRequest()
   {
      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket();
      BipedContinuousPlanningRequestPacket continuousPlanningRequestPacket = this.planningRequestPacket.get();

      plannerStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      SideDependentList<Pose3DReadOnly> startPositions = new SideDependentList<>();
      if (isInitialSegmentOfPlan.getBooleanValue())
      {
         /*
         if (robotDataReceiver != null)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               FramePose3D position = new FramePoint3D(robotDataReceiver.getReferenceFrames().getSoleFrame(robotSide));
               position.changeFrame(ReferenceFrame.getWorldFrame());

               startPositions.put(robotSide, position);
            }
         }
         else
         {
         */
            Pose3D leftFoot = new Pose3D();
            leftFoot.setPosition(continuousPlanningRequestPacket.getLeftStartPositionInWorld());
            leftFoot.setOrientation(continuousPlanningRequestPacket.getLeftStartOrientationInWorld());
            Pose3D rightFoot = new Pose3D();
            rightFoot.setPosition(continuousPlanningRequestPacket.getRightStartPositionInWorld());
            rightFoot.setOrientation(continuousPlanningRequestPacket.getRightStartOrientationInWorld());

            startPositions.put(RobotSide.LEFT, leftFoot);
            startPositions.put(RobotSide.RIGHT, rightFoot);
//         }

         for (RobotSide robotSide : RobotSide.values)
            currentFeetPositions.put(robotSide, startPositions.get(robotSide));
      }
      else
      {
         int numberOfFixedSteps = fixedStepQueue.size();
         if (fixedStepQueue.size() > 2)
         {
            Pose3D footPose = new Pose3D();
            FootstepDataMessage footstep = fixedStepQueue.get(numberOfFixedSteps - 2);
            footPose.setPosition(footstep.getLocation());
            footPose.setOrientation(footstep.getOrientation());
            currentFeetPositions.put(RobotSide.fromByte(footstep.getRobotSide()), footPose);
         }

         if (fixedStepQueue.size() > 1)
         {
            Pose3D footPose = new Pose3D();
            FootstepDataMessage footstep = fixedStepQueue.get(numberOfFixedSteps - 1);
            footPose.setPosition(footstep.getLocation());
            footPose.setOrientation(footstep.getOrientation());
            currentFeetPositions.put(RobotSide.fromByte(footstep.getRobotSide()), footPose);
         }
      }

      if (fixedStepQueue.size() > 0)
      {
         FootstepDataMessage finalStep = fixedStepQueue.get(fixedStepQueue.size() - 1);
         expectedInitialSteppingSide.set(RobotSide.fromByte(finalStep.getRobotSide()).getOppositeSide());
      }

      RobotSide initialSupportSide = expectedInitialSteppingSide.get().getOppositeSide();
      Pose3DReadOnly stancePose = currentFeetPositions.get(initialSupportSide);

      planningRequestPacket.setInitialStanceRobotSide(initialSupportSide.toByte());
      planningRequestPacket.setHorizonLength(continuousPlanningRequestPacket.getHorizonLength());
      planningRequestPacket.setRequestedFootstepPlannerType(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR.toByte());
      planningRequestPacket.getStanceFootPositionInWorld().set(stancePose.getPosition());
      planningRequestPacket.getStanceFootOrientationInWorld().set(stancePose.getOrientation());
      planningRequestPacket.setTimeout(continuousPlanningRequestPacket.getTimeout());
//      planningRequestPacket.setBestEffortTimeout(continuousPlanningRequestPacket.getBestEffortTimeout());
      planningRequestPacket.getGoalPositionInWorld().set(continuousPlanningRequestPacket.getGoalPositionInWorld());
      planningRequestPacket.getGoalOrientationInWorld().set(continuousPlanningRequestPacket.getGoalOrientationInWorld());
      planningRequestPacket.getPlanarRegionsListMessage().set(latestPlanarRegions.get());

      broadcastPlanId.increment();
      planningRequestPacket.setPlannerRequestId(broadcastPlanId.getIntegerValue());

      if (debug || verbose)
      {
         LogTools.info("Planning #" + broadcastPlanId.getValue() + " to " + continuousPlanningRequestPacket.getGoalPositionInWorld() + ", starting with " + expectedInitialSteppingSide
               .get());
      }
      waitingForPlan.set(true);
      planningRequestPublisher.publish(planningRequestPacket);
   }


   private boolean clearCompletedSteps()
   {
      List<FootstepStatusMessage> statusMessages = this.footstepStatusBuffer.getAndSet(new ArrayList<>());

      for (FootstepStatusMessage footstepStatusMessage : statusMessages)
      {
         if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            FootstepDataMessage stepToRemove = fixedStepQueue.remove(0);
            if (stepToRemove.getRobotSide() != footstepStatusMessage.getRobotSide())
            {
               LogTools.warn("The completed step was not the next step in the queue. Killing the module.");
               resetForNewPlan();
               planningFailed.set(true);

               return false;
            }
         }
      }

//      this.footstepStatusBuffer.set(new ArrayList<>());

      return true;
   }

   private void checkIfLastPlanFulfilledRequest()
   {
      FootstepPlanningToolboxOutputStatus plannerOutput = currentPlannerOutput.get();
      if (plannerOutput == null)
      {
         receivedPlanForLastRequest.set(false);
         return;
      }

      receivedPlanForLastRequest.set(broadcastPlanId.getIntegerValue() == receivedPlanId.getIntegerValue());
   }

   private boolean updateStepQueueFromNewPlan()
   {
//      FootstepPlanningToolboxOutputStatus plannerOutput = latestPlannerOutput.getAndSet(null);
      FootstepPlanningToolboxOutputStatus plannerOutput = latestPlannerOutput.get();
      if (plannerOutput == null)
         return false;

      // ignore this plan, it's out of date
      if (plannerOutput.getPlanId() != broadcastPlanId.getValue())
         return false;

      currentPlannerOutput.set(plannerOutput);
      List<FootstepDataMessage> stepList = plannerOutput.getFootstepDataList().getFootstepDataList();
      stepQueue.clear();
      for (int i = 0; i < stepList.size(); i++)
      {
         FootstepDataMessage step = new FootstepDataMessage(stepList.get(i));
         step.setTransferDuration(transferDuration.getDoubleValue());
         step.setSwingDuration(swingDuration.getDoubleValue());
         stepQueue.add(step);
      }

      if (stepQueue.get(0).getRobotSide() != expectedInitialSteppingSide.get().toByte())
      {
         LogTools.error("Got a plan back starting with the wrong first step. Should start with " + expectedInitialSteppingSide.get() + ", but actually starts with " +
                              RobotSide.fromByte(stepQueue.get(0).getRobotSide()));
         resetForNewPlan();
         planningFailed.set(true);

         return false;
      }

//      latestPlannerOutput.set(null);
      isInitialSegmentOfPlan.set(false);

      return true;
   }

   private boolean updateFixedStepQueue()
   {
      if (fixedStepQueue.size() > numberOfStepsToLeaveUnchanged.getValue() || stepQueue.size() == 0)
         return false;

      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue())
      {
         FootstepDataMessage footstepToAdd = stepQueue.remove(0);
         if (fixedStepQueue.size() > 0)
         {
            byte lastSide = fixedStepQueue.get(fixedStepQueue.size() - 1).getRobotSide();
            if (lastSide == footstepToAdd.getRobotSide())
            {
               LogTools.error("The next quadrant when appending would be out of order. The fixed queue size " + fixedStepQueue.size() +
                                    " ends with a step on the " + RobotSide.fromByte(lastSide) + ", meaning it should start with " +
                                    RobotSide.fromByte(lastSide).getOppositeSide() + ". \nIt actually starts with " +
                                    RobotSide.fromByte(footstepToAdd.getRobotSide()) + ", but " + expectedInitialSteppingSide.get() + " was requested for starting.");
               resetForNewPlan();
               planningFailed.set(true);

               return false;
            }
         }
         fixedStepQueue.add(footstepToAdd);
      }

      return true;
   }

   private void broadcastLatestPlan()
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      // TODO make an output status thing and report it
      for (int i = 0; i < fixedStepQueue.size(); i++)
         footstepDataListMessage.getFootstepDataList().add().set(fixedStepQueue.get(i));
      for (int i = 0; i < stepQueue.size(); i++)
         footstepDataListMessage.getFootstepDataList().add().set(stepQueue.get(i));

      reportMessage(footstepDataListMessage);
   }

   private void updateHasReachedGoal()
   {
      boolean isPlannedGoalTheFinalGoal = false;
      if (currentPlannerOutput.get() != null)
      {
         isPlannedGoalTheFinalGoal = currentPlannerOutput.get().getLowLevelPlannerGoal().getPosition().distanceXY(planningRequestPacket.get().getGoalPositionInWorld()) < proximityGoal;
      }
      if (!isInitialSegmentOfPlan.getBooleanValue() && isPlannedGoalTheFinalGoal && stepQueue.isEmpty())
         hasReachedGoal.set(true);
      else
         hasReachedGoal.set(false);
   }
}
