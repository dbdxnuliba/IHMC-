package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.log.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.messager.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.ros2.*;

import java.util.concurrent.*;

// this is behavior that should be purely called from the another behavior and will not have the ability to triggered from the ui.
// if you need a reset for some reason embedded in your behavior and write your own :|
public class ResetRobotPoseBehavior
{

   private final BehaviorHelper behaviorHelper;
   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private Messager messager;
   private DRCRobotModel drcRobotModel;
   private Ros2Node ros2Node;
//   private final AtomicReference<Boolean> enable;

   private boolean trigger1 = false;
   private boolean trigger2 = false;
   private boolean trigger3 = false;
   private Notification reset = new Notification();

   private double resetRobotTime = 3.0; // reduce this after tuning

   public ResetRobotPoseBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      this.behaviorHelper = behaviorHelper;
      this.messager = messager;
      this.ros2Node = ros2Node;

      this.drcRobotModel = robotModel;

//      enable = messager.createInput(API.Enable, false);
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


      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::resetRobot, 1, TimeUnit.SECONDS);

      setReset();

   }

   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();

      if(tmp.getTrajectoryExecutionStatus() == tmp.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         System.out.println("Done resetting :" + tmp.getEndEffectorName());
         if(tmp.getEndEffectorNameAsString().equals("pelvis"))
         {
            trigger3 = true;
         }
      }
   }

   private void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
   {
      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();

      if(tmp.getTrajectoryExecutionStatus() == tmp.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         System.out.println("Done Resetting :" + tmp.getJointNames().getString(0));
         if(tmp.getJointNames().getString(0).equals("l_arm_shz"))
         {
            trigger1 = true;
         }

      }
   }

   int i = 1;
   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
      WalkingStatusMessage tmp = message.takeNextData();


      if(tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         System.out.println("FootTrajectoryStatusMessage Triggered");
         if(i ==1)
         {
            trigger2 = true;
         }

         i++;
      }
   }

   public void resetRobot()
   {
      if(reset.poll())
      {
         System.out.println("Resetting robot");
         fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();

//         behaviorHelper.publishFootstepList(walk());
         double[] jointAngles = new double[] {0.0, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
         behaviorHelper.requestArmTrajectory(RobotSide.LEFT, 2.0, jointAngles);

         jointAngles = new double[] {0.0, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
         behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, 2.0, jointAngles);


      }

      if(trigger1)
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstepDataListMessage.getFootstepDataList();
//         for(RobotSide side: RobotSide.values)
//         {
            MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(RobotSide.LEFT);
            FramePoint3D footLocation = new FramePoint3D(stepFrame);//, x, side.negateIfRightSide(y), z);
            FrameQuaternion footOrienation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrienation.changeFrame(ReferenceFrame.getWorldFrame());

//            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, footLocation, footOrienation);
//            footstepDataMessages.add().set(footstepDataMessage);

            footLocation = new FramePoint3D(stepFrame, footLocation.getX(), - footLocation.getY(), footLocation.getZ());
            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, footLocation, footOrienation);
            footstepDataMessages.add().set(footstepDataMessage);
      //            x = x + 0.4;
//         }

         behaviorHelper.publishFootstepList(footstepDataListMessage);
         trigger1 = false;


      }

      else if (trigger2)
      {

//         behaviorHelper.requestPelvisTrajectory(2.0,);
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(),
                                                          behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());

         double pelvisTrajectoryTime = 0.5;
         FramePose3D pelvisZUp = new FramePose3D();
         pelvisZUp.setFromReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());
         FramePoint3D pelvis_1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), pelvisZUp.getX(),pelvisZUp.getY(),pelvisZUp.getZ());
         FrameQuaternion orientation = new FrameQuaternion();
         orientation.setYawPitchRoll(0,0.0,0.0);
         behaviorHelper.requestPelvisTrajectory(pelvisTrajectoryTime,pelvis_1, orientation);
         trigger2 = false;
      }

      else if (trigger3)
      {
         doOnAbort(true);
         trigger3 = false;
      }



   }

   private void doOnAbort(boolean flag)
   {
      if (flag)
      {
         LogTools.info("Done Resetting Robot");
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

//      for (int i = 0; i < 5; i++)
//      {
         for(RobotSide side: RobotSide.values)
         {
            MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
            FramePoint3D footLocation = new FramePoint3D(stepFrame, x, side.negateIfRightSide(y), z);
            FrameQuaternion footOrienation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrienation.changeFrame(ReferenceFrame.getWorldFrame());

            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrienation);
            footstepDataMessages.add().set(footstepDataMessage);
//            x = x + 0.4;
         }
//      }
      footstepDataListMessage.setAreFootstepsAdjustable(true);
      return footstepDataListMessage;
   }

   public void setReset()
   {
      reset.set();
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Behaviors");
      private static final MessagerAPIFactory.CategoryTheme reset = apiFactory.createCategoryTheme("ResetRobot");
   }

}

