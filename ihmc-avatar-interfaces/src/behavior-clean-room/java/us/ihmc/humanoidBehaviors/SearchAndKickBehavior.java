package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
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

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class SearchAndKickBehavior
{

   private final BehaviorHelper behaviorHelper;
   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private final AtomicReference<Boolean> enable;
   private SuppaKickBehavior kick;
   private Messager messager;
   private DRCRobotModel robotModel;
   private Ros2Node ros2Node;
   private ResetRobotPoseBehavior resetRobotPoseBehavior;
//   private newEventBasedBehaviorSample tmp;

   private newSuppaKickBehavior suppaKickBehavior;
   private ArrayList<BehaviorAction> ActionBehaviors;

   private int behaviorCounter = 0;
   private boolean flag = false;

   private final Notification flagNotification = new Notification();
   private final Notification WalkNotification = new Notification();
   private final Notification taskspaceNotification = new Notification();
   private final Notification jointSpaceNotification = new Notification();
   private final Notification walkingNotification = new Notification();

   private final Notification goToWalk = new Notification();
   private final Notification doOnlyOnce = new Notification();

   private final boolean triggerfromAnotherBehavior = true;



   public SearchAndKickBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      LogTools.debug("Initializing SearchAndKickBehavior");
      this.behaviorHelper = behaviorHelper;
      this.messager = messager;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;

      messager.registerTopicListener(API.Abort, this::doOnAbort);
      messager.registerTopicListener(API.SearchAndKick, object -> WalkNotification.set());

//      goToWalk.set();
      doOnlyOnce.set();

      new BehaviorBuilder(action, actionTypes.Footstep);

      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);

      if(triggerfromAnotherBehavior)
      {
         enable = new AtomicReference<Boolean>(true);
         goToWalk.set();
      }
      else
      {
         enable = messager.createInput(API.Enable, false);
      }


   }

//   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
//   {
//      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if(tmp.getTrajectoryExecutionStatus() == tmp.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
//      {
//         System.out.println(tmp.getEndEffectorName());
//      }
//   }
//
//   private void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
//   {
//      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if(tmp.getTrajectoryExecutionStatus() == tmp.getTrajectoryExecutionStatus())
//      {
//         System.out.println(tmp.getJointNames().getString(0));
//      }
//   }
//
//   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
//   {
//      WalkingStatusMessage tmp = message.takeNextData();
//
//
//      if(tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
//      {
//         System.out.println("FootTrajectoryStatusMessage Triggered");
//         flag = true;
//         flagNotification.set();
//
//      }
//   }



//   private void doBehavior()
//   {
////      System.out.println(WalkNotification.read());
//      if (!enable.get())
//      {
//         return;
//      }
//
//      if (flagNotification.poll())
//      {
//         // walk upto ball and kick
//
//         //         behaviorHelper.publishFootstepList(walk());
////         behaviorHelper.startScheduledThread();
////         kick = new SuppaKickBehavior(behaviorHelper, messager, robotModel, ros2Node);
////         resetRobotPoseBehavior = new ResetRobotPoseBehavior(behaviorHelper, messager, robotModel,ros2Node);
////         behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::running, 1, TimeUnit.SECONDS);
////         System.out.println("In walk Notification method");
//         flag = false;
////         kick.setGoToWalk();
//      }
//
//      if (WalkNotification.poll())
//      {
//         fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
////         behaviorHelper.publishFootstepList(walk());
////         resetRobotPoseBehavior = new ResetRobotPoseBehavior(behaviorHelper, messager, robotModel,ros2Node);
////         tmp = new newEventBasedBehaviorSample(behaviorHelper, messager, robotModel, ros2Node);
//         suppaKickBehavior = new  newSuppaKickBehavior(behaviorHelper, messager, robotModel, ros2Node);
//      }
//   }

   BehaviorAction action = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {
         behaviorHelper.publishFootstepList(walk());
      }
   };

   BehaviorAction action1 = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

      }
   };

   BehaviorAction action2 = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

      }
   };


   BehaviorAction action3 = new BehaviorAction()
   {
      @Override
      public void onEntry()
      {

      }
   };

   public void doBehavior()
   {
//      System.out.println(WalkNotification.read());
      if(!enable.get())
      {
         return;
      }

      if(doOnlyOnce.poll())
      {
         System.out.println("inside doBehavior loop");
         ActionBehaviors = BehaviorBuilder.getActionsBehavior();
      }

      if(WalkNotification.poll())
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
      System.out.println("inside walk method");
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstepDataListMessage.getFootstepDataList();
      fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
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
      private static final Category RootCategory = apiFactory.createRootCategory("Behaviors");
      private static final CategoryTheme Kicking = apiFactory.createCategoryTheme("LookibfAndSearching");
      private static final Category KickingCategory = RootCategory.child(Kicking);
//      private static final MessagerAPIFactory.Category SearchAndKickCategory = RootCategory.child()

      public static final Topic<Boolean> Enable = KickingCategory.topic(apiFactory.createTypedTopicTheme("enable"));
      public static final Topic<Boolean> SearchAndKick = KickingCategory.topic(apiFactory.createTypedTopicTheme("Initialize Behavior"));
      public static final Topic<Boolean> Abort = KickingCategory.topic(apiFactory.createTypedTopicTheme("Abort Behavior"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
