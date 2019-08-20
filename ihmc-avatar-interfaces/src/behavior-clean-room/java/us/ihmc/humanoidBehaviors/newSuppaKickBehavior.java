package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.humanoidBehaviors.BehaviorBuilder.*;
import us.ihmc.humanoidBehaviors.BehaviorBuilder.BehaviorBuilder.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.humanoidBehaviors.newEventBasedBehaviorSample.*;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.log.*;
import us.ihmc.messager.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.*;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class newSuppaKickBehavior
{
   private final BehaviorHelper behaviorHelper;
   private Ros2Node ros2Node;
   private DRCRobotModel robotModel;

   private AtomicReference<Boolean> enable;
   private final ActivationReference<Boolean> stepping;
   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private ArrayList<BehaviorAction> ActionBehaviors;
   private int behaviorCounter = 0;

   private final Notification goToWalk = new Notification();
//   private final Notification taskspaceNotification = new Notification();
   private final Notification jointSpaceNotification = new Notification();
   private final Notification walkingNotification = new Notification();
   private final Notification doOnlyOnce = new Notification();

   private final Notification pelvistaskspaceNotification = new Notification();
   private final Notification chesttaskspaceNotification = new Notification();
   private final Notification rlegtaskspaceNotification = new Notification();
   private final Notification llegtaskspaceNotification = new Notification();

   private ResetRobotPoseBehavior resetRobotPoseBehavior;

   public newSuppaKickBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
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

      new BehaviorBuilder(getArmsBack, actionTypes.LeftArm);
      new BehaviorBuilder(getPelvisup, actionTypes.Pelvis);
      new BehaviorBuilder(getRightFootUp, actionTypes.RightLeg);
      new BehaviorBuilder(ChestForwardandFootAction, actionTypes.Chest, actionTypes.RightLeg);
      new BehaviorBuilder(MidBehavFootAction, actionTypes.RightLeg);
      new BehaviorBuilder(ChestBackandArmsandFootAction,actionTypes.Chest , actionTypes.LeftArm, actionTypes.RightLeg);
      new BehaviorBuilder(doNothing, actionTypes.RightLeg);
      new BehaviorBuilder(FinalAction,actionTypes.Chest, actionTypes.RightLeg);


      goToWalk.set();
      doOnlyOnce.set();
      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);
   }

   BehaviorAction getPelvisup = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         HumanoidReferenceFrames humanoidReferenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
         System.out.println("Doing pelvis motion");
         double pelvisTrajectoryTime = 0.50;
         FramePose3D pelvisZUp = new FramePose3D();
         pelvisZUp.setFromReferenceFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         FramePoint3D pelvis_1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), pelvisZUp.getX(),pelvisZUp.getY()+0.1,pelvisZUp.getZ());
         FrameQuaternion orientation = new FrameQuaternion();
         orientation.setYawPitchRoll(0,0.0,0.0);
         behaviorHelper.requestPelvisTrajectory(pelvisTrajectoryTime,pelvis_1, orientation);
      }
   };

   BehaviorAction getArmsBack = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         System.out.println("Doing Arms motion");
         double armTrajectoryTime = 0.5;
         double[] jointAngles = new double[] {Math.toRadians(15.0),Math.toRadians(-90.0),Math.toRadians(90.0),Math.toRadians(0.0),
               Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
         behaviorHelper.requestArmTrajectory(RobotSide.LEFT, armTrajectoryTime, jointAngles);
         jointAngles = new double[] {0.0,Math.toRadians(90.0),0.0,0.0,0.0,0.0,0.0};
         behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, armTrajectoryTime, jointAngles);
      }
   };

   BehaviorAction getRightFootUp = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         HumanoidReferenceFrames humanoidReferenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(0.0,-0.25,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);

      }
   };



   BehaviorAction ChestForwardandFootAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         System.out.println("Doing chest motion");
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(30.0), 0.0);
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(),
                                                          behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());

         HumanoidReferenceFrames humanoidReferenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(-0.35,-0.15,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
      }
   };

   BehaviorAction MidBehavFootAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         HumanoidReferenceFrames humanoidReferenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(-0.35,-0.15,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
      }
   };

   BehaviorAction ChestBackandArmsandFootAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

         System.out.println("Doing chest motion");
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(-10.0), 0.0);
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(),
                                                          behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());

         System.out.println("Doing Arms motion");
         double armTrajectoryTime = 0.5;
         double[] jointAngles = new double[] {Math.toRadians(0.0),Math.toRadians(-90.0),Math.toRadians(75.0),Math.toRadians(60.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
         behaviorHelper.requestArmTrajectory(RobotSide.LEFT, armTrajectoryTime, jointAngles);
         jointAngles = new double[] {0.0,Math.toRadians(90.0),Math.toRadians(75.0),Math.toRadians(-60.0),0.0,0.0,0.0};
         behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, armTrajectoryTime, jointAngles);

         FramePose3D tmp = new FramePose3D();
         HumanoidReferenceFrames humanoidReferenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(0.3,-0.15,0.1,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);

      }
   };

   BehaviorAction FinalAction = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         System.out.println("Final Foot Action");
         FramePose3D tmp = new FramePose3D();
         HumanoidReferenceFrames humanoidReferenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

         behaviorHelper.requestChestGoHome(0.5);
         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(0.3,-0.15,0.0,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
      }
   };

   BehaviorAction doNothing = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
      }
   };


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

   private void triggerAppropriateListener(List<BehaviorBuilder.actionTypes>  actions)
   {
      for(int i = 0; i < actions.size(); ++i)
      {
         if (i == actions.size() -1)
         {
            if(actions.get(i).equals(actionTypes.LeftLeg))

            {
               llegtaskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForLeftLeg);
            }

            else if(actions.get(i).equals(actionTypes.RightLeg))
            {
               rlegtaskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForRightLeg);

            }

            else if (actions.get(i).equals(actionTypes.Chest))
            {
               chesttaskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessageForChest);
            }

            else if (actions.get(i).equals(actionTypes.Pelvis))
            {
               pelvistaskspaceNotification.set();
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
            if(chesttaskspaceNotification.poll())
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
            if(llegtaskspaceNotification.poll())
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
            if(rlegtaskspaceNotification.poll())
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
            if(pelvistaskspaceNotification.poll())
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

   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         behaviorHelper.shutdownScheduledThread();
      }
   }
}
