package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import org.lwjgl.*;
import org.omg.PortableInterceptor.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.communication.net.*;
import us.ihmc.communication.packets.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.humanoidBehaviors.behaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior.*;
import us.ihmc.humanoidBehaviors.taskExecutor.*;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.log.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.taskExecutor.*;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.*;

import java.sql.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class SuppaKickBehavior
{
//   private final PipeLine<AbstractBehavior> pipeLine;
   private final BehaviorHelper behaviorHelper;
   private final ActivationReference<Boolean> stepping;
   private final AtomicReference<Boolean> enable;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);

   public SuppaKickBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel)
   {
      LogTools.debug("Initializing SearchAndKickBehavior");
//      flag = behaviorHelper.createBooleanActivationReference(API.Walk,false,true);
      this.behaviorHelper = behaviorHelper;

      behaviorHelper.createFootstepStatusCallback(this::acceptFootstepStatus);
      stepping = behaviorHelper.createBooleanActivationReference(API.Stepping, false, true);
//      messager.registerTopicListener(API.Abort,this::doOnAbort);
//      messager.registerTopicListener(API.Walk, object -> goToWalk.set());
//      controllerState = new ROS2Input<>(ros2Node, HighLevelStateChangeStatusMessage.class, robotName, controllerId, initialState, this::acceptStatusChange);

//      ROS2Tools.createCallbackSubscription(ros2Node, ToolboxStateMessage.class, getSubscriberTopicNameGenerator(), s -> receivedPacket(s.takeNextData()));
      enable = messager.createInput(API.Enable, false);

      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);

   }

//   public void receivedPacket(ToolboxStateMessage message)
//   {
////      if (DEBUG)
////         LogTools.info("Received a state message.");
////
////      if (toolboxTaskScheduled != null)
////      {
////         return;
////      }
//
//      switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
//      {
//      case WAKE_UP:
//         wakeUp();
//         break;
//      case REINITIALIZE:
//         reinitialize();
//         break;
//      case SLEEP:
//         sleep();
//         break;
//      }
//   }

//   public void wakeUp()
//   {
//      if (toolboxTaskScheduled != null)
//      {
//         if (DEBUG)
//            LogTools.error("This toolbox is already running.");
//         return;
//      }
//
//      if (DEBUG)
//         LogTools.debug("Waking up");
//
//      createToolboxRunnable();
//      toolboxTaskScheduled = executorService.scheduleAtFixedRate(toolboxRunnable, 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);
//      getToolboxController().setFutureToListenTo(toolboxTaskScheduled);
//      reinitialize();
//      receivedInput.set(true);
//   }
//
//   private void reinitialize()
//   {
//      getToolboxController().requestInitialize();
//   }
//
//   public void sleep()
//   {
//
//      if (DEBUG)
//         LogTools.debug("Going to sleep");
//
//      destroyToolboxRunnable();
//
//      if (toolboxTaskScheduled == null)
//      {
//         if (DEBUG)
//            LogTools.error("There is no task running.");
//         return;
//      }
//
//      getToolboxController().setFutureToListenTo(null);
//      toolboxTaskScheduled.cancel(true);
//      toolboxTaskScheduled = null;
//   }
//
//   private void doOnAbort(boolean abort)
//   {
//      if (abort)
//      {
//         LogTools.info("Abort received. Shutting down threadScheduler.");
//         behaviorHelper.shutdownScheduledThread();
//      }
//   }

   private void acceptFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      if (!enable.get())
         return;

      LogTools.info("acceptFootstepStatus: " + footstepStatusMessage);

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         int footstepsTakenSoFar = footstepsTaken.incrementAndGet();
         LogTools.info("Have taken " + footstepsTakenSoFar + " footsteps.");
      }
   }

   private void doBehavior()
   {
      if(!enable.get())
      {
         return;
      }

      if(stepping.poll())
      {
         if(stepping.hasChanged())
         {
            LogTools.info("Sending Steps");
         }

         if (footstepsTaken.compareAndSet(2,0))
         {
            LogTools.info("Sending Steps");

            FullHumanoidRobotModel fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
            FootstepDataListMessage foorStepList = createTwoStepInPlaceSteps(fullHumanoidRobotModel);
            behaviorHelper.publishFootstepList(foorStepList);
         }

      }

      else if (stepping.hasChanged())
      {
         LogTools.info("Stopped Stepping");
      }
   }

   private FootstepDataListMessage createTwoStepInPlaceSteps(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      FootstepDataListMessage footstelpList = new FootstepDataListMessage();
      RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstelpList.getFootstepDataList();

      double x = 0;
      double y = -0.10;
      double z = 0;
      for(RobotSide side : RobotSide.values())
      {
//         RobotSide tmpside = RobotSide.LEFT;
         MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
         FramePoint3D footLocation = new FramePoint3D(stepFrame);
         FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
         footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrientation);
         footstepDataMessages.add().set(footstepDataMessage);
         x = x + 0.4;
      }
      footstelpList.setAreFootstepsAdjustable(true);
      return footstelpList;
   }

//   public HighLevelControllerName latestControllerState()
//   {
//      return HighLevelControllerName.fromByte(controllerState.getLatest().getEndHighLevelControllerName());
//   }


   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("SearchAndKickBehavior");
      private static final CategoryTheme SearchAndKick = apiFactory.createCategoryTheme("KickThings");
      private static final Category SearchAndKickCategory = RootCategory.child(SearchAndKick);

      public static final Topic<Boolean> Stepping =  SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("Stepping"));
      public static final Topic<Boolean> Enable =  SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("Enable"));
      public static final Topic<Boolean> Abort =  SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("Abort"));
      public static final Topic<Boolean> Walk = SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("walking12345"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
