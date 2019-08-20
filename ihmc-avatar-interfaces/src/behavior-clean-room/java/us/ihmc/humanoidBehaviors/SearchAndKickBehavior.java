package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.log.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.ros2.*;

import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class SearchAndKickBehavior
{

   private final BehaviorHelper behaviorHelper;
   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private final AtomicReference<Boolean> enable;
   private SuppaKickBehavior kick;
   private Messager messager;
   private DRCRobotModel drcRobotModel;
   private Ros2Node ros2Node;
   private ResetRobotPoseBehavior resetRobotPoseBehavior;
   private newEventBasedBehaviorSample tmp;


   private int behaviorCounter = 1;
   private boolean flag = false;

   private final Notification flagNotification = new Notification();
   private final Notification WalkNotification = new Notification();

   public SearchAndKickBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      LogTools.debug("Initializing SearchAndKickBehavior");
      this.behaviorHelper = behaviorHelper;
      this.messager = messager;
      this.drcRobotModel = robotModel;
      this.ros2Node = ros2Node;



      enable = messager.createInput(API.Enable, false);
      messager.registerTopicListener(API.Abort, this::doOnAbort);
      messager.registerTopicListener(API.SearchAndKick, object -> WalkNotification.set());
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           WalkingStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkFootTrajectoryMessage);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           JointspaceTrajectoryStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkJointTrajectoryMessage);



      ROS2Tools.createCallbackSubscription(ros2Node,
                                           TaskspaceTrajectoryStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkTaskspaceTrajectoryMessage);


      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);

   }

   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();

      if(tmp.getTrajectoryExecutionStatus() == tmp.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         System.out.println(tmp.getEndEffectorName());
      }
   }

   private void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
   {
      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();

      if(tmp.getTrajectoryExecutionStatus() == tmp.getTrajectoryExecutionStatus())
      {
         System.out.println(tmp.getJointNames().getString(0));
      }
   }

   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
      WalkingStatusMessage tmp = message.takeNextData();


      if(tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         System.out.println("FootTrajectoryStatusMessage Triggered");
         flag = true;
         flagNotification.set();

      }
   }


   private void doBehavior()
   {
//      System.out.println(WalkNotification.read());
      if (!enable.get())
      {
         return;
      }

      if (flagNotification.poll())
      {
         // walk upto ball and kick

         //         behaviorHelper.publishFootstepList(walk());
//         behaviorHelper.startScheduledThread();
//         kick = new SuppaKickBehavior(behaviorHelper, messager, drcRobotModel, ros2Node);
//         resetRobotPoseBehavior = new ResetRobotPoseBehavior(behaviorHelper, messager, drcRobotModel,ros2Node);
//         behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::running, 1, TimeUnit.SECONDS);
//         System.out.println("In walk Notification method");
         flag = false;
//         kick.setGoToWalk();
      }

      if (WalkNotification.poll())
      {
         fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
//         behaviorHelper.publishFootstepList(walk());
//         resetRobotPoseBehavior = new ResetRobotPoseBehavior(behaviorHelper, messager, drcRobotModel,ros2Node);
         tmp = new newEventBasedBehaviorSample(behaviorHelper, messager, drcRobotModel, ros2Node);
      }
   }

   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         behaviorHelper.shutdownScheduledThread();
      }
   }



   public FootstepDataListMessage walk()
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstepDataListMessage.getFootstepDataList();

      double x = 0.0;
      double y = 0.09;
      double z = 0.0;

      for (int i = 0; i < 5; i++)
      {
         for(RobotSide side: RobotSide.values)
         {
            MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
            FramePoint3D footLocation = new FramePoint3D(stepFrame, x,side.negateIfRightSide(y),z);
            FrameQuaternion footOrienation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrienation.changeFrame(ReferenceFrame.getWorldFrame());

            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrienation);
            footstepDataMessages.add().set(footstepDataMessage);
            x = x + 0.4;
         }
      }
      footstepDataListMessage.setAreFootstepsAdjustable(true);
      return footstepDataListMessage;
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Behaviors");
      private static final MessagerAPIFactory.CategoryTheme Kicking = apiFactory.createCategoryTheme("LookibfAndSearching");
//      private static final MessagerAPIFactory.Category SearchAndKickCategory = RootCategory.child()

      public static final Topic<Boolean> Enable = RootCategory.topic(apiFactory.createTypedTopicTheme("enable"));
      public static final Topic<Boolean> SearchAndKick = RootCategory.topic(apiFactory.createTypedTopicTheme("Initialize Behavior"));
      public static final Topic<Boolean> Abort = RootCategory.topic(apiFactory.createTypedTopicTheme("Abort Behavior"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
