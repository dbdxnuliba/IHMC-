package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.humanoidBehaviors.BehaviorBuilder.*;
import us.ihmc.humanoidBehaviors.BehaviorBuilder.BehaviorBuilder.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
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
import us.ihmc.tools.thread.*;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class newEventBasedBehaviorSample
{

   private final BehaviorHelper behaviorHelper;
   private final ActivationReference<Boolean> stepping;
   private AtomicReference<Boolean> enable;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);
   private FullHumanoidRobotModel fullHumanoidRobotModel;


   private Ros2Node ros2Node;
   private DRCRobotModel robotModel;
   private ArrayList<BehaviorAction>  ActionBehaviors;

   private int behaviorCounter = 0;



   private final Notification goToWalk = new Notification();
   private final Notification taskspaceNotification = new Notification();
   private final Notification jointSpaceNotification = new Notification();
   private final Notification walkingNotification = new Notification();
   private final Notification doOnlyOnce = new Notification();



   BehaviorAction multipleActions = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

         System.out.println("Doing chest motion");
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(0.0), 0.0);
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(), behaviorHelper.pollHumanoidReferenceFrames().getChestFrame());

         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(-0.35,-0.15,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);

         double pelvisTrajectoryTime = 1.0;
         FramePose3D pelvisZUp = new FramePose3D();
         pelvisZUp.setFromReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());
         FramePoint3D pelvis_1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), pelvisZUp.getX(),pelvisZUp.getY(),pelvisZUp.getZ());
         FrameQuaternion orientation = new FrameQuaternion();
         orientation.setYawPitchRoll(Math.toRadians(0),0.0,0.0);
         behaviorHelper.requestPelvisTrajectory(pelvisTrajectoryTime,pelvis_1, orientation);
      }
   };

   BehaviorAction chestAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

         System.out.println("Doing chest motion");
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(0.0), 0.0);
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(), behaviorHelper.pollHumanoidReferenceFrames().getChestFrame());

//         System.out.println("Doing Arms motion");
//         double armTrajectoryTime = 0.5;
//         double[] jointAngles = new double[] {Math.toRadians(0.0),Math.toRadians(-90.0),Math.toRadians(75.0),Math.toRadians(60.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
//         behaviorHelper.requestArmTrajectory(RobotSide.LEFT, armTrajectoryTime, jointAngles);
//         jointAngles = new double[] {0.0,Math.toRadians(90.0),Math.toRadians(75.0),Math.toRadians(-60.0),0.0,0.0,0.0};
//         behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, armTrajectoryTime, jointAngles);
      }
   };

   BehaviorAction chestAction1 = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

         System.out.println("Doing chest motion");
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, Math.toRadians(0.0), 0.0);
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,
                                                          chestOrientation,
                                                          ReferenceFrame.getWorldFrame(),
                                                          behaviorHelper.pollHumanoidReferenceFrames().getChestFrame());
      }
   };

   BehaviorAction armsAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         System.out.println("Doing Arms motion");
         double armTrajectoryTime = 1.0;
         double[] jointAngles = new double[] {Math.toRadians(15.0),Math.toRadians(-90.0),Math.toRadians(90.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
         behaviorHelper.requestArmTrajectory(RobotSide.LEFT, armTrajectoryTime, jointAngles);
         jointAngles = new double[] {0.0,Math.toRadians(90.0),0.0,0.0,0.0,0.0,0.0};
         behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, armTrajectoryTime, jointAngles);
      }
   };

   BehaviorAction pelvisAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         System.out.println("Doing pelvis motion");
         double pelvisTrajectoryTime = 1.0;
         FramePose3D pelvisZUp = new FramePose3D();
         pelvisZUp.setFromReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());
         FramePoint3D pelvis_1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), pelvisZUp.getX(),pelvisZUp.getY(),pelvisZUp.getZ());
         FrameQuaternion orientation = new FrameQuaternion();
         orientation.setYawPitchRoll(0,0.0,0.0);
         behaviorHelper.requestPelvisTrajectory(pelvisTrajectoryTime,pelvis_1, orientation);
      }
   };

   BehaviorAction pelvisAction1 = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         System.out.println("Doing pelvis1 motion");
         double pelvisTrajectoryTime = 1.0;
         FramePose3D pelvisZUp = new FramePose3D();
         pelvisZUp.setFromReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());
         FramePoint3D pelvis_1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), pelvisZUp.getX(),pelvisZUp.getY(),pelvisZUp.getZ());
         FrameQuaternion orientation = new FrameQuaternion();
         orientation.setYawPitchRoll(Math.toRadians(55),0.0,0.0);
         behaviorHelper.requestPelvisTrajectory(pelvisTrajectoryTime,pelvis_1, orientation);
      }
   };



   BehaviorAction leftLEgAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getAnkleZUpFrame(RobotSide.LEFT));
         tmp.set(new Pose3D(0.0, 0.25,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.LEFT,1.0,tmp);
      }
   };

   BehaviorAction rightLegAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(0.0, -0.25,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,1.0,tmp);
      }
   };

   BehaviorAction walking = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         FootstepDataListMessage footstepList = new FootstepDataListMessage();
         RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstepList.getFootstepDataList();

         RobotSide side  = RobotSide.LEFT;

            MovingReferenceFrame stepFrame = behaviorHelper.pollFullRobotModel().getSoleFrame(side);
            FramePoint3D footLocation = new FramePoint3D(stepFrame);
            FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation);
            footstepDataMessages.add().set(footstepDataMessage);

            stepFrame = behaviorHelper.pollFullRobotModel().getSoleFrame(side.getOppositeSide());
            footLocation = new FramePoint3D(stepFrame);
            footLocation.setY(-0.11);
            footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side.getOppositeSide(), footLocation, footOrientation);
            footstepDataMessages.add().set(footstepDataMessage);

         footstepList.setAreFootstepsAdjustable(true);
         behaviorHelper.publishFootstepList(footstepList);
      }
   };



   public newEventBasedBehaviorSample(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      LogTools.debug("Initializing SuppaKickBehavior");

      this.behaviorHelper = behaviorHelper;
      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      stepping = behaviorHelper.createBooleanActivationReference(API.Stepping, false, true);
      messager.registerTopicListener(API.Abort,this::doOnAbort);
//      messager.registerTopicListener(API.Walk, object -> goToWalk.set()); //triggers the notification class set method (like a ping)
      fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();



      //go to walk is the name of the bahavior itself
      enable = messager.createInput(API.Enable, false);



//      BehaviorBuilder build1 = new BehaviorBuilder(actionTypes.Chest, multipleActions );
      BehaviorBuilder build2 = new BehaviorBuilder(armsAction, actionTypes.LeftArm); // both arms
      BehaviorBuilder build3 = new BehaviorBuilder(walking, actionTypes.Footstep);
//      BehaviorBuilder build4 = new BehaviorBuilder(multipleActions,actionTypes.Chest, actionTypes.RightLeg,  actionTypes.Pelvis);
//      BehaviorBuilder build5 = new BehaviorBuilder(pelvisAction, actionTypes.Pelvis);
//      BehaviorBuilder build6 = new BehaviorBuilder(pelvisAction1, actionTypes.Pelvis);
//      BehaviorBuilder build7 = new BehaviorBuilder(pelvisAction1, actionTypes.Pelvis);
//      BehaviorBuilder build8 = new BehaviorBuilder(rightLegAction, actionTypes.RightLeg);

      goToWalk.set();
      doOnlyOnce.set();
      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);

   }



   private void triggerAppropriateListener(List<BehaviorBuilder.actionTypes>  actions)
   {
      for(int i = 0; i < actions.size(); ++i)
      {
         if (i == actions.size() -1)
         {
            if(actions.get(i).equals(actionTypes.LeftLeg))

            {
               taskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForLeftLeg);
            }

            else if(actions.get(i).equals(actionTypes.RightLeg))
            {
               taskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForRightLeg);

            }

            else if (actions.get(i).equals(actionTypes.Chest))
            {
               taskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForChest);
            }

            else if (actions.get(i).equals(actionTypes.Pelvis))
            {
               taskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForPelvis);
            }
            else if(actions.get(i).equals(actionTypes.LeftArm) || actions.get(i).equals(actionTypes.RightArm))
            {
               jointSpaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    JointspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkJointTrajectoryMessage);
            }
            else
            {
               walkingNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    WalkingStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkFootTrajectoryMessage);
            }
         }
      }
   }



   public void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
   {
      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();
      jointSpaceNotification.set();
      if (tmp.getTrajectoryExecutionStatus() == JointspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         if(jointSpaceNotification.poll())
         {
            if(tmp.getJointNames().getString(0).equals("r_arm_shz"))
            {
               if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
               {
                  if(ActionBehaviors.get(behaviorCounter).isDone())
                  {
                     behaviorCounter++;
                     goToWalk.set();
                  }
               }

            }
         }
      }
   }


   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
      WalkingStatusMessage tmp = message.takeNextData();

      if (tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         if(walkingNotification.poll())
         {
            if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
            {
               ActionBehaviors.get(behaviorCounter).isDone();
               behaviorCounter++;
            }
            goToWalk.set();
         }
      }
   }


   private void checkTaskspaceTrajectoryMessageForChest(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         if(tmp.getEndEffectorNameAsString().equals("utorso"))
         {
            if(taskspaceNotification.poll())
            {
               if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
               {
                  System.out.println("Executing Done method for :" + ActionBehaviors.get(behaviorCounter));
                  if (ActionBehaviors.get(behaviorCounter).isDone())
                  {
                     behaviorCounter++;
                  }
               }
               goToWalk.set();
            }
         }
      }
   }

   private void checkTaskspaceTrajectoryMessageForLeftLeg(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         if(tmp.getEndEffectorNameAsString().equals("l_foot"))
         {
            if(taskspaceNotification.poll())
            {
               if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
               {
                  System.out.println("Executing Done method for :" + ActionBehaviors.get(behaviorCounter));
                  if (ActionBehaviors.get(behaviorCounter).isDone())
                  {
                     behaviorCounter++;
                  }
               }
               goToWalk.set();
            }
         }
      }
   }

   private void checkTaskspaceTrajectoryMessageForRightLeg(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         if(tmp.getEndEffectorNameAsString().equals("r_foot"))
         {
            if(taskspaceNotification.poll())
            {
               if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
               {
                  System.out.println("Executing Done method for :" + ActionBehaviors.get(behaviorCounter));
                  if (ActionBehaviors.get(behaviorCounter).isDone())
                  {
                     behaviorCounter++;
                  }
               }
               goToWalk.set();
            }
         }
      }
   }


   private void checkTaskspaceTrajectoryMessageForPelvis(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
      {
         if(tmp.getEndEffectorNameAsString().equals("pelvis"))
         {
            if(taskspaceNotification.poll())
            {
               if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
               {
                  System.out.println("Executing Done method for :" + ActionBehaviors.get(behaviorCounter));
                  if (ActionBehaviors.get(behaviorCounter).isDone())
                  {
                     behaviorCounter++;
                  }
               }
               goToWalk.set();
            }
         }
      }
   }


   public void doBehavior()
   {
      if(doOnlyOnce.poll())
      {
         System.out.println("inside newEventBasedBehaviorSample doBehavior loop");
         ActionBehaviors = BehaviorBuilder.getActionsBehavior();
      }

      if(goToWalk.poll())
      {

         System.out.println("Triggering Behavior"  + (behaviorCounter));
         if(behaviorCounter == ActionBehaviors.size())
         {
            doOnAbort(true);
         }
         ActionBehaviors.get(behaviorCounter).onEntry();
         System.out.println(BehaviorBuilder.getActionsTypeList());
         triggerAppropriateListener(BehaviorBuilder.getActionsTypeList().get(behaviorCounter));
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
