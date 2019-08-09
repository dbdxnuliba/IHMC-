package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import org.lwjgl.*;
import org.omg.PortableInterceptor.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.communication.net.*;
import us.ihmc.communication.packets.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.humanoidBehaviors.behaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior.*;
import us.ihmc.humanoidBehaviors.taskExecutor.*;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidBehaviors.utilities.*;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.communication.subscribers.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.log.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.taskExecutor.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.tools.thread.*;
import us.ihmc.yoVariables.registry.*;
import us.ihmc.yoVariables.variable.*;

import java.sql.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class SuppaKickBehavior
{

   private final BehaviorHelper behaviorHelper;
   private final ActivationReference<Boolean> stepping;
   private final AtomicReference<Boolean> enable;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry  = new YoGraphicsListRegistry();


   private boolean pelvisFlag = false;
   private boolean leftArmFlag = false;
   private boolean rightArmFlag = false;
   private boolean chestFlag = false;
   private boolean footStepFlag = false;
   private boolean LeftLegFlag = false;
   private boolean RightLegFlag = false;


   private final Notification goToWalk = new Notification();
   private boolean flag1 = false;



   public SuppaKickBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      LogTools.debug("Initializing SearchAndKickBehavior");

      this.behaviorHelper = behaviorHelper;

      behaviorHelper.createFootstepStatusCallback(this::acceptFootstepStatus);
      stepping = behaviorHelper.createBooleanActivationReference(API.Stepping, false, true);
      messager.registerTopicListener(API.Abort,this::doOnAbort);
      messager.registerTopicListener(API.Walk, object -> goToWalk.set()); //triggers the notification class set method (like a ping)

      enable = messager.createInput(API.Enable, false);

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
//      System.out.println("Task Space End - effector Name" +message.takeNextData().getEndEffectorNameAsString());
//      System.out.println(message.takeNextData().getTrajectoryExecutionStatus());
      if(message.takeNextData().getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         System.out.println("Pelvis executed");
         pelvisFlag = true;
      }
   }
   double counter = 0;
   private void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
   {
      //      System.out.println("Walking Status : " + message.takeNextData().getWalkingStatus());
//      System.out.println(message.takeNextData().getTimestamp());
//      System.out.println("Actual Joint Position" + message.takeNextData().getActualJointPositions());// + "Desired Joint Position" +
//                               message.readNextData().getDesiredJointPositions());
      if(message.takeNextData().getTrajectoryExecutionStatus() == JointspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_STARTED)
      {
         System.out.println("I am in the joint space trajectory loop and done with hand movements" + counter);
         counter++;
      }
   }


   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
//      System.out.println("Walking Status : " + message.takeNextData().getWalkingStatus());

      if(message.takeNextData().getWalkingStatus() == 1)
      {
         flag1 = true;
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

      if(goToWalk.poll())
      {
//         thread.start();
         LogTools.info("Walking few steps");
         FullHumanoidRobotModel fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
         FootstepDataListMessage footstepDataListMessage = gotoWalk(fullHumanoidRobotModel);
         behaviorHelper.publishFootstepList(footstepDataListMessage);
//         System.out.println(behaviorHelper.getLatestControllerState());
      }

      if(enable.get())
      {
         if(flag1)
         {
//            System.out.println(longTime);
            FullHumanoidRobotModel fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
//            FootstepDataListMessage footstepDataListMessage = nowTurn(fullHumanoidRobotModel);
            nowTurn(fullHumanoidRobotModel);

//            behaviorHelper.publishFootstepList(nowTurn());
         }
      }
   }

//   public FootstepDataListMessage nowTurn(FullHumanoidRobotModel fullHumanoidRobotModel)
   public void nowTurn(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
//      FootstepDataListMessage footstelpList = new FootstepDataListMessage();
//      for (RobotSide side : RobotSide.values())
//      {
//         MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
//         FramePoint3D footLocation = new FramePoint3D(stepFrame);
//         FrameQuaternion footOrientation = new FrameQuaternion(stepFrame,90.0,0.0,0.0);
//         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
//         footOrientation.changeFrame(ReferenceFrame.getWorldFrame());
//
//         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrientation);
//         footstelpList.getFootstepDataList().add().set(footstepDataMessage);
//
//      }
//      return footstelpList;
      double trajectoryTime = 1.0;
//      behaviorHelper.requestChestGoHome(trajectoryTime);
//      behaviorHelper.requestPelvisGoHome(trajectoryTime);

      double[] jointAngles = new double[] {0.0, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
      behaviorHelper.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {0.0, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
      behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);

      FramePoint3D pelvisPOs = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0,0.0,9.1);
      FrameQuaternion pelvisOrientation = new FrameQuaternion();
      behaviorHelper.requestPelvisTrajectory(trajectoryTime,pelvisPOs,pelvisOrientation);
      flag1 = false;

   }


   public FootstepDataListMessage gotoWalk(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      double x = 0.0;
      double y = 0.11;
      double z = 0.0;

      FootstepDataListMessage footstelpList = new FootstepDataListMessage();
      for(int i = 0; i < 5; ++i)
      {
         for (RobotSide side : RobotSide.values())
         {
            MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
            FramePoint3D footLocation = new FramePoint3D(stepFrame,x, side.negateIfRightSide(y),z);
            FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrientation);
            footstelpList.getFootstepDataList().add().set(footstepDataMessage);
            x = x + 0.4;
         }
      }
      footstelpList.setAreFootstepsAdjustable(true);
      return footstelpList;
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

   public static void main(String[] args)
   {
//      BehaviorHelper tmp = new BehaviorHelper();
//      new SuppaKickBehavior();
   }
}
