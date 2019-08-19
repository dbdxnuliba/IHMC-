package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import org.apache.poi.ss.formula.functions.*;
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
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.log.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.pubsub.attributes.*;
import us.ihmc.pubsub.common.*;
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
   private final ActivationReference<Boolean> stepping;
   private AtomicReference<Boolean> enable;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);
   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private ArrayList<BehaviorBuilder.actionTypes> actionTypesArrayList;
   private ArrayList<List<BehaviorBuilder.actionTypes>> newList;
//   private BehaviorBuilder forRunnable;
   private Ros2Node ros2Node;
   private DRCRobotModel robotModel;
   private ArrayList<BehaviorAction>  ActionBehaviors;

   private int behaviorCounter = 0;
   private int tempActionSize;


//   private boolean pelvisFlag = false;
//   private boolean leftArmFlag = false;
//   private boolean rightArmFlag = false;
//   private boolean chestFlag = false;
//   private boolean footStepFlag = false;
//   private boolean LeftLegFlag = false;
//   private boolean RightLegFlag = false;

//   private boolean startBehavior;
   //   private ArrayList<Boolean> pelvisFlags = new ArrayList<>(Arrays.asList(false,false,false,false,false));
   //   private ArrayList<Boolean> leftArmFlags = new ArrayList<>(Arrays.asList(false,false,false,false,false));
   //   private ArrayList<Boolean> rightArmFlags = new ArrayList<>(Arrays.asList(false,false,false,false,false));
   //   private ArrayList<Boolean> legLegFlags = new ArrayList<>(Arrays.asList(false,false,false,false,false));
   //   private ArrayList<Boolean> rightLegFlags = new ArrayList<>(Arrays.asList(false,false,false,false,false));
   //   private ArrayList<Boolean> chestFlags = new ArrayList<>(Arrays.asList(false,false,false,false,false));

   private final Notification goToWalk = new Notification();
   private final Notification taskspaceNotification = new Notification();
   private final Notification jointSpaceNotification = new Notification();
   private final Notification walkingNotification = new Notification();
   private final Notification doOnlyOnce = new Notification();
   private final Notification subtract = new Notification();
//   private boolean tmp = false;
   private final boolean triggerfromAnotherBehavior = false;

   private PausablePeriodicThread taskSpaceThread;

   BehaviorAction multipleActions = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

         System.out.println("Doing chest motion");
         double chestTrajectoryTime = 0.5;
         FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
         chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(55.0), 0.0);
         behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(), behaviorHelper.pollHumanoidReferenceFrames().getChestFrame());

         FramePose3D tmp = new FramePose3D();
         tmp.setReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getAnkleZUpFrame(RobotSide.RIGHT));
         tmp.set(new Pose3D(-0.35,-0.15,0.127,0.0,0.0,0.0));
         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
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

            footLocation = new FramePoint3D(stepFrame);
            footLocation.setY(-footLocation.getY());
            footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side.getOppositeSide(), footLocation, footOrientation);
            footstepDataMessages.add().set(footstepDataMessage);

         footstepList.setAreFootstepsAdjustable(true);
         behaviorHelper.publishFootstepList(footstepList);
      }
   };



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


//      ROS2Tools.createCallbackSubscription(ros2Node,
//                                           WalkingStatusMessage.class,
//                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
//                                           this::checkFootTrajectoryMessage);
//
//      ROS2Tools.createCallbackSubscription(ros2Node,
//                                           JointspaceTrajectoryStatusMessage.class,
//                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
//                                           this::checkJointTrajectoryMessage);
//
//
//
//      ROS2Tools.createCallbackSubscription(ros2Node,
//                                           TaskspaceTrajectoryStatusMessage.class,
//                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
//                                           this::checkTaskspaceTrajectoryMessage);




      //      goToWalk.set();
//      if(triggerfromAnotherBehavior)
//      {
//         enable = new AtomicReference<Boolean>(true);
//         goToWalk.set();
//      }
      // current they are being added in reverse.
//      BehaviorBuilder build6 = new BehaviorBuilder(actionTypes.LeftLeg,leftLEgAction);
//      BehaviorBuilder build7 = new BehaviorBuilder(actionTypes.Pelvis,pelvisAction2);
//      BehaviorBuilder build6 = new BehaviorBuilder(actionTypes.Chest, chestAction1 );//, ros2Node, robotModel);
//      BehaviorBuilder build5 = new BehaviorBuilder(actionTypes.Pelvis,pelvisAction1);
//      BehaviorBuilder build8 = new BehaviorBuilder(actionTypes.Footstep, walking);

      BehaviorBuilder build2 = new BehaviorBuilder(chestAction, actionTypes.Chest);//, ros2Node, robotModel);
//      BehaviorBuilder build1 = new BehaviorBuilder(actionTypes.Chest, multipleActions );
//      BehaviorBuilder build2 = new BehaviorBuilder(actionTypes.LeftArm,armsAction); // both arms
//      BehaviorBuilder build4 = new BehaviorBuilder(leftLEgAction, actionTypes.LeftLeg);
      BehaviorBuilder build3 = new BehaviorBuilder(pelvisAction, actionTypes.Pelvis);
      BehaviorBuilder build1 = new BehaviorBuilder(rightLegAction, actionTypes.RightLeg);

//      taskSpaceThread = new PausablePeriodicThread(this::triggerAppropriateListener, 0.1, "");
      goToWalk.set();
      doOnlyOnce.set();
      //, ros2Node, robotModel) ;
//      forRunnable = new BehaviorBuilder(actionTypes.Pelvis,armsAction, ros2Node, robotModel);
      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);

   }

   // footstep counter acts like a counter for sequence of tasks

//   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
//   {
//
//      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if(tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
//      {
//         if (tmp.getEndEffectorNameAsString().equals("r_foot"))
//         {
//            if(footStepCounter == 2)
//            {
//               chestFlag = true;
//            }
//            else if (footStepCounter == 3)
//            {
//               footStepFlag= true;
//            }
//            else if(footStepCounter == 4)
//            {
//               chestFlag = true;
//            }
//
//            else if (footStepCounter == 6)
//            {
//               onBehaviorDone();
//            }
//
//         }
//
//         if (tmp.getEndEffectorNameAsString().equals("pelvis"))
//         {
//            leftArmFlag = true;
//            rightArmFlag = true;
//         }
//      }
//   }

//   private void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
//   {
//      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if(tmp.getTrajectoryExecutionStatus() == JointspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_STARTED)
//      {
//         if (tmp.getJointNames().getString(0).equals("l_arm_shz"))// || tmp.getJointNames().getString(0).equals("r_arm_shz"))
//         {
//            footStepFlag = true;
//         }
//      }
//   }


//   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
//   {
//      WalkingStatusMessage tmp = message.takeNextData();
//
//
//      if(tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
//      {
//         System.out.println("FootTrajectoryStatusMessage Triggered");
//
//      }
//   }


// if pelvis action then check jointTrajectoryStatusMessage


//   private void triggerAppropriateListener(ArrayList<BehaviorBuilder.actionTypes> actions)
   private void triggerAppropriateListener(List<BehaviorBuilder.actionTypes>  actions)
   {
      if(actions.size() == 1)
      {
         if(actions.get(0).equals(actionTypes.LeftLeg) || actions.get(0).equals(actionTypes.Pelvis) || actions.get(0).equals(actionTypes.RightLeg) || actions.get(0).equals(actionTypes.Chest))
         {
            taskspaceNotification.set();
            ROS2Tools.createCallbackSubscription(ros2Node,
                                                 TaskspaceTrajectoryStatusMessage.class,
                                                 ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                 this::checkTaskspaceTrajectoryMessage);
         }
         else if(actions.get(0).equals(actionTypes.LeftArm) || actions.get(0).equals(actionTypes.RightArm))
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

      else
      {
         System.out.println("Work in progress");

         for(int i = 0; i < actions.size(); ++i)
         {
            if(actions.get(i).equals(actionTypes.LeftLeg) || actions.get(i).equals(actionTypes.Pelvis) || actions.get(i).equals(actionTypes.RightLeg) ||
                  actions.get(i).equals(actionTypes.Chest))
            {
               taskspaceNotification.set();
               ROS2Tools.createCallbackSubscription(ros2Node,
                                                    TaskspaceTrajectoryStatusMessage.class,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                                    this::checkTaskspaceTrajectoryMessage);
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

//   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
//   {
//
//      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
//      {
////         BehaviorBuilder.getActionsBehavior().get(behaviorCounter).isDone();
////         ActionBehaviors.get(behaviorCounter).isDone();
//         if(tmp.getEndEffectorNameAsString().equals("l_foot"))
//         {
//            System.out.println("Am I printed twice???");
//            if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
//            {
//               ActionBehaviors.get(behaviorCounter).isDone();
//               behaviorCounter++;
//            }
//            goToWalk.set();
//         }
//
//      }
//
//   }


//   private void checkTaskspaceTrajectoryMessagePelvis(Subscriber<TaskspaceTrajectoryStatusMessage> message)
//   {
//
//      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
//      {
//         if(pelvisNotification.poll())
//         {
//            tmp.setTrajectoryExecutionStatus(TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_STARTED);
//            if (tmp.getEndEffectorNameAsString().equals("pelvis"))
//            {
//
//               if (behaviorCounter != ActionBehaviors.size()) // don't call get after the last action is done as it will be out of bounds
//               {
//                  System.out.println("Executing Done method for :" + ActionBehaviors.get(behaviorCounter));
//                  if (ActionBehaviors.get(behaviorCounter).isDone())
//                  {
//                     behaviorCounter++;
//                  }
//
//               }
//               goToWalk.set();
//               //            taskSpaceThread.stop();
//
//            }
//
//         }
//         //         BehaviorBuilder.getActionsBehavior().get(behaviorCounter).isDone();
//         //         ActionBehaviors.get(behaviorCounter).isDone();
//
//      }
//
//   }

   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
         TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
         if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
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

   public void doBehavior()
   {
      if(doOnlyOnce.poll())
      {
         System.out.println("inside newSuppaKickBehavior doBehavior loop");
//         actionTypesArrayList =  BehaviorBuilder.getActionTypes();
         newList = BehaviorBuilder.getActionsTypeList();
         ActionBehaviors = BehaviorBuilder.getActionsBehavior();
      }

      if(goToWalk.poll())
      {
         if(subtract.poll())
         {
            behaviorCounter = behaviorCounter - (tempActionSize -1);
         }


         System.out.println("Triggering Behavior"  + (behaviorCounter));
         if(behaviorCounter == ActionBehaviors.size())
         {
            doOnAbort(true);
         }
         ActionBehaviors.get(behaviorCounter).onEntry();
         // pass in the counter for the previous behavior or the previous behavior type
//         System.out.println("Triggering behavior" + BehaviorBuilder.getActionTypes().get(behaviorCounter));
//         triggerAppropriateListener(BehaviorBuilder.getActionTypes().get(behaviorCounter));
         triggerAppropriateListener(BehaviorBuilder.getActionsTypeList().get(behaviorCounter));



         if (BehaviorBuilder.getActionsTypeList().get(behaviorCounter).size() > 1)
         {
            tempActionSize = BehaviorBuilder.getActionsTypeList().get(behaviorCounter).size();
            subtract.set();
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

//   public void doBehavior()
//   {
//      //      System.out.println("NOw in doBehavior loop");
//      if(!enable.get())
//      {
//         return;
//      }
//
//      if(stepping.poll())
//      {
//         if(stepping.hasChanged())
//         {
//            LogTools.info("Sending Steps");
//         }
//      }
//
//      else if (stepping.hasChanged())
//      {
//         LogTools.info("Stopped Stepping");
//      }
//
//
//
//      if(goToWalk.poll())
//      {
//
//         System.out.println("Inside doBehavior and goToWalk poll");
//         getPelvisUp(behaviorHelper.pollHumanoidReferenceFrames());
//         startBehavior = true;
//      }
//
//
//      if (startBehavior)
//      {
//         // write a helper class that build sequence of flags for you given as an input a sequence of actions
//         // given a pelvis action with as #1 it add the first flag as pelvis
//         // #2 getArmsBack -> armsFlag
//         // #3 chestForward() and submitFootPosition() -> either chestFlag or WalkingStatusFLag and so on
//         if(leftArmFlag && rightArmFlag)
//         {
//            getArmsBack();
//         }
//
//         if(chestFlag)
//         {
//
//            if(footStepCounter ==2)
//            {
//               Chestforward();
//               submitFootPostion(behaviorHelper.pollHumanoidReferenceFrames(), footStepCounter);
//               footStepCounter++;
//            }
//
//            else if(footStepCounter == 4)
//            {
//               Chestback();
//               getArmsUp();
//               submitFootPostion(behaviorHelper.pollHumanoidReferenceFrames(), footStepCounter);
//               footStepCounter++;
//               submitFootPostion(behaviorHelper.pollHumanoidReferenceFrames(), footStepCounter);
//               footStepCounter++;
//            }
//
//         }
//
//         if(footStepFlag)
//         {
//            if (footStepCounter == 1)
//            {
//               submitFootPostion(behaviorHelper.pollHumanoidReferenceFrames(), footStepCounter);
//               footStepCounter++;
//            }
//
//            if(footStepCounter == 3)
//            {
//               submitFootPostion(behaviorHelper.pollHumanoidReferenceFrames(),footStepCounter);
//               footStepCounter++;
//            }
//
//         }
//      }
//
//   }

//   private void resetRobotPose()
//   {
//      double goHomeTime = 2.0;
//      behaviorHelper.requestChestGoHome(goHomeTime);
//      behaviorHelper.requestPelvisGoHome(goHomeTime);
//      behaviorHelper.requestArmsGoHome(goHomeTime);
//
//   }
//
//   private void onBehaviorDone()
//   {
//      double goHomeTime = 1.0;
//      behaviorHelper.requestChestGoHome(goHomeTime);
//      behaviorHelper.requestPelvisGoHome(goHomeTime);
//      FramePose3D tmp = new FramePose3D();
//      tmp.setReferenceFrame(behaviorHelper.pollHumanoidReferenceFrames().getAnkleZUpFrame(RobotSide.RIGHT));
//      tmp.set(new Pose3D(0.0,-0.11,0.0,0.0,0.0,0.0));
//      behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,goHomeTime,tmp);
//      startBehavior = false;
//   }
//
//   private void Chestforward()
//   {
//      System.out.println("Doing chest motion");
//      double chestTrajectoryTime = 0.5;
//      FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
//      chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(30.0), 0.0);
//      behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(), behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());
//      chestFlag = false;
//   }
//
//   private void Chestback()
//   {
//      System.out.println("Doing chest motion");
//      double chestTrajectoryTime = 0.5;
//      FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
//      chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(),0.0, Math.toRadians(-10.0), 0.0);
//      behaviorHelper.requestChestOrientationTrajectory(chestTrajectoryTime,chestOrientation, ReferenceFrame.getWorldFrame(), behaviorHelper.pollHumanoidReferenceFrames().getPelvisZUpFrame());
//      chestFlag = false;
//   }
//
//   public void getPelvisUp(HumanoidReferenceFrames humanoidReferenceFrames)
//   {
//
//      System.out.println("Doing pelvis motion");
//      double pelvisTrajectoryTime = 1.50;
//      FramePose3D pelvisZUp = new FramePose3D();
//      pelvisZUp.setFromReferenceFrame(humanoidReferenceFrames.getPelvisZUpFrame());
//      FramePoint3D pelvis_1 = new FramePoint3D(ReferenceFrame.getWorldFrame(), pelvisZUp.getX(),pelvisZUp.getY()+0.1,pelvisZUp.getZ());
//      FrameQuaternion orientation = new FrameQuaternion();
//      orientation.setYawPitchRoll(0,0.0,0.0);
//      behaviorHelper.requestPelvisTrajectory(pelvisTrajectoryTime,pelvis_1, orientation);
//      pelvisFlag = false;
//
//   }
//
//
//   public void getArmsBack()
//   {
//      System.out.println("Doing Arms motion");
//      double armTrajectoryTime = 0.5;
//      double[] jointAngles = new double[] {Math.toRadians(15.0),Math.toRadians(-90.0),Math.toRadians(90.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
//      behaviorHelper.requestArmTrajectory(RobotSide.LEFT, armTrajectoryTime, jointAngles);
//      jointAngles = new double[] {0.0,Math.toRadians(90.0),0.0,0.0,0.0,0.0,0.0};
//      behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, armTrajectoryTime, jointAngles);
//      leftArmFlag = false;
//      rightArmFlag = false;
//   }
//
//   public void getArmsUp()
//   {
//      System.out.println("Doing Arms motion");
//      double armTrajectoryTime = 0.5;
//      double[] jointAngles = new double[] {Math.toRadians(0.0),Math.toRadians(-90.0),Math.toRadians(75.0),Math.toRadians(60.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
//      behaviorHelper.requestArmTrajectory(RobotSide.LEFT, armTrajectoryTime, jointAngles);
//      jointAngles = new double[] {0.0,Math.toRadians(90.0),Math.toRadians(75.0),Math.toRadians(-60.0),0.0,0.0,0.0};
//      behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, armTrajectoryTime, jointAngles);
//   }
//
//   //   3
//   public void submitFootPostion(HumanoidReferenceFrames humanoidReferenceFrames, int counter)
//   {
//      FramePose3D tmp = new FramePose3D();
//
//      if (counter == 1)
//      {
//         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
//         tmp.set(new Pose3D(0.0,-0.25,0.127,0.0,0.0,0.0));
//         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
//         footStepFlag = false;
//      }
//      else if (counter == 2)
//      {
//         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
//         tmp.set(new Pose3D(-0.35,-0.15,0.127,0.0,0.0,0.0));
//         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
//         chestFlag = false; // redundant but safe measure
//      }
//      else if (counter == 3)
//      {
//         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
//         tmp.set(new Pose3D(-0.35,-0.15,0.127,0.0,0.0,0.0));
//         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
//         footStepFlag = false;
//      }
//
//      else if (counter == 4)
//      {
//         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
//         tmp.set(new Pose3D(0.3,-0.15,0.05,0.0,0.0,0.0));
//         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
//         chestFlag = false;
//      }
//      else if (counter == 5)
//      {
//         tmp.setReferenceFrame(humanoidReferenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
//         tmp.set(new Pose3D(0.0,-0.25,0.127,0.0,0.0,0.0));
//         behaviorHelper.requestFootTrajectory(RobotSide.RIGHT,0.5,tmp);
//         chestFlag = false;
//      }
//
//   }
//
//   public void setGoToWalk()
//
//   {
//      //      System.out.println("Setting Walking Notification");
//      //      goToWalk.poll();
//      //      System.out.println(goToWalk.read());
//      //      enable = new AtomicReference<Boolean>(true);
//      //      goToWalk.set();
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
