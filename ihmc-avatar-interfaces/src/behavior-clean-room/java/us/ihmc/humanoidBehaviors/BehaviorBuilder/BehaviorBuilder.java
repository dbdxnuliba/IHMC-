package us.ihmc.humanoidBehaviors.BehaviorBuilder;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.communication.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.ros2.*;

import java.util.*;

public class BehaviorBuilder
{
   public enum actionTypes
   {
      Chest,
      Pelvis,
      LeftArm,
      RightArm,
      LeftLeg,
      RightLeg,
      Footstep
   }


   private int typeCounter = 0;
   private int behaviorCounter = 0;
   private static ArrayList<BehaviorBuilder.actionTypes> actionsType = new ArrayList<>(100);
   private static ArrayList<BehaviorAction> actionsBehavior = new ArrayList<>(100);
   private static ArrayList<BehaviorBuilder.actionTypes> flags = new ArrayList<>();
   private BehaviorBuilder.actionTypes type;
   private BehaviorAction behaviorAction;


   public BehaviorBuilder(actionTypes type, BehaviorAction behaviorAction)//, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
//      actions = new ArrayList<BehaviorAction>();
//      for(int i = 0 ; i < behaviorAction.length; i++)
//      {
//         behaviorAction1 = behaviorAction[i];
//         helperMethod(behaviorAction1);
//      }

      this.type = type;
      this.behaviorAction = behaviorAction;
      parentHelperMethod();

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

   }

   private void parentHelperMethod()
   {
      helperMethod(type);
      helperMethodforactions(behaviorAction);

   }

   private void helperMethodforactions(BehaviorAction action)
   {
//      actions.get(counter);
      actionsBehavior.add(behaviorCounter, action);
      behaviorCounter++;
   }
   private void helperMethod(BehaviorBuilder.actionTypes type)
   {
      //      actions.get(counter);
      actionsType.add(typeCounter, type);
      typeCounter++;
   }

   public static ArrayList<BehaviorAction> getActionsBehavior()
   {
      return actionsBehavior;
   }

   public static ArrayList<BehaviorBuilder.actionTypes> getActionTypes()
   {
      return actionsType;
   }

//   chest -> 1
//   pelvis -> 2
//   LeftArm -> 3
//   RightArm -> 4
//   LeftLeg -> 5
//   LeftLeg -> 6
//   RightLeg -> 7

   private static ArrayList<BehaviorBuilder.actionTypes> buildFlags()
   {
      ArrayList<BehaviorBuilder.actionTypes> listforFlags =  getActionTypes();
      for(int i = 0; i< listforFlags.size() ; i ++)
      {
         if (listforFlags.get(i).equals(actionTypes.Chest))
         {
            flags.add(i, actionTypes.Chest);
         }
         else if (listforFlags.get(i).equals(actionTypes.Pelvis))
         {
            flags.add(i, actionTypes.Pelvis);
         }

         else if (listforFlags.get(i).equals(actionTypes.LeftArm))
         {
            flags.add(i, actionTypes.LeftArm);
         }

         else if (listforFlags.get(i).equals(actionTypes.RightArm))
         {
            flags.add(i, actionTypes.RightArm);
         }
         else if (listforFlags.get(i).equals(actionTypes.LeftLeg))
         {
            flags.add(i, actionTypes.LeftLeg);
         }
         else if (listforFlags.get(i).equals(actionTypes.RightLeg))
         {
            flags.add(i, actionTypes.RightLeg);
         }
      }

      return flags;
   }

//   public void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
//   {
//      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if (tmp.getTrajectoryExecutionStatus() == JointspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_STARTED);
//
//   }
//
//   public void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
//   {
//      WalkingStatusMessage tmp = message.takeNextData();
//
//      if (tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED);
//   }
//
//   public void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
//   {
//
//      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();
//
//      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
//         ;
//   }

//   public static void main(String[] args)
//   {
//      BehaviorAction action1 = new BehaviorAction()
//      {
//         @Override
//         public void onEntry()
//         {
//
//         }
//      };
//
//      BehaviorAction action2 = new BehaviorAction()
//      {
//         @Override
//         public void onEntry()
//         {
//
//         }
//      };
//
//
//      BehaviorBuilder build1 = new BehaviorBuilder(actionTypes.Pelvis, action1);// , ros2Node, robotModel);
//      BehaviorBuilder build2 = new BehaviorBuilder(actionTypes.LeftArm,action2);//, ros2Node, robotModel) ;
//
//      BehaviorBuilder build1 = new BehaviorBuilder(actionTypes.Chest,action1);
//      BehaviorBuilder build2 = new BehaviorBuilder(actionTypes.Pelvis,action2);
//      BehaviorBuilder build3 = new BehaviorBuilder(actionTypes.Pelvis,action2);
//      BehaviorBuilder build2 = new BehaviorBuilder( actionTypes.Pelvis,action2);
//      ArrayList<Integer> flug =  BehaviorBuilder.buildFlags();
//      ArrayList<BehaviorBuilder.actionTypes> flug =  BehaviorBuilder.buildFlags();
//      System.out.println(BehaviorBuilder.getActionTypes());
//      System.out.println(BehaviorBuilder.getActionsBehavior());
//
//
//
//   }
}
