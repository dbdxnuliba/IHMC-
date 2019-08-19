package us.ihmc.humanoidBehaviors.BehaviorBuilder;

import controller_msgs.msg.dds.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.pubsub.subscriber.*;

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


   private static int typeCounter = 0;
   private int behaviorCounter = 0;
   private static ArrayList<BehaviorBuilder.actionTypes> actionsType = new ArrayList<>(100);
   private static ArrayList<List<actionTypes>> actionsTypeList = new ArrayList<>();
   private static ArrayList<List<actionTypes>> finaList = new ArrayList<>();
//   private static List<actionTypes> actionsTypeList = new ArrayList<>();

   private static ArrayList<BehaviorAction> actionsBehavior = new ArrayList<>(100);
   private static ArrayList<BehaviorBuilder.actionTypes> flags = new ArrayList<>();
   private static final Map<BehaviorAction,actionTypes> executionMap = new LinkedHashMap<>();
   private static final ArrayDeque<actionTypes> KeyQueue = new ArrayDeque<>();

   private BehaviorBuilder.actionTypes  type;
//   private static final ArrayDeque<BehaviorBuilder.actionTypes> taskQueue = new ArrayDeque<>();
   private static final ArrayDeque<BehaviorAction> taskQueue = new ArrayDeque<>();
   private BehaviorAction behaviorAction;


//   public BehaviorBuilder(BehaviorAction behaviorAction,actionTypes... type)//, Ros2Node ros2Node, DRCRobotModel robotModel)
   public BehaviorBuilder(BehaviorAction behaviorAction,actionTypes... type)
   {
//      actions = new ArrayList<BehaviorAction>();
//      for(int i = 0 ; i < behaviorAction.length; i++)
//      {
//         behaviorAction1 = behaviorAction[i];
//         helperMethod(behaviorAction1);
//      }
      for(int i = 0; i< type.length ; i++)
      {
         this.type = type[i];
      }
      this.behaviorAction = behaviorAction;
//      if (type.length == 1)
//      {
//         helperMethodforactiontype(type[0]);
//      }
//
//      else
//      {
      buildActionsTypeList(type.length , type);
      helperMethodforactions(behaviorAction);
//      buildFinalList(type);


      //      parentHelperMethod();

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


//   private void parentHelperMethod()
//   {
////      for(int i = 0; i < type ; i++)
////      {
////         helperMethodforactiontype(type);
////      }
//
//      helperMethodforactions(behaviorAction);
//
//
//   }

   private void helperMethodforactions(BehaviorAction action)
   {
//      actions.get(counter);
      actionsBehavior.add(behaviorCounter, action);
      behaviorCounter++;
   }
//   private void helperMethodforactiontype(BehaviorBuilder.actionTypes type)
//   {
//      //      actions.get(counter);
//      actionsType.add(typeCounter, type);
//
//      typeCounter++;
//
//   }


   private static void buildActionsTypeList(int number, actionTypes... actionsType)
   {
      ArrayList<actionTypes> tmp = new ArrayList<>();

      for(int i = 0 ; i < number; ++i)
      {
         tmp.add(actionsType[i]);
      }
      actionsTypeList.add(tmp);
   }

   //   private static void submitSingleTask(BehaviorBuilder.actionTypes type)
//   private static void submitSingleTask(BehaviorAction behaviorAction, actionTypes type)
//   {
//      taskQueue.add(behaviorAction);
//      buildtaskType(type);
//   }

   private static void buildtaskType(actionTypes key)
   {
      KeyQueue.add(key);
   }

//   private static void submitTaskForParallelExecution(ArrayList<List<actionTypes>> abc)
//   {
////      actionTypes type =  executionMap.get(key);
////      //      BehaviorBuilder tmp = new BehaviorBuilder();
////      //      tmp.submitSingleTask();
////      submitSingleTask(action, key);
//
//      actionsTypeList.add(abc);
//
//
//   }






   public static ArrayDeque<BehaviorAction> getTaskQueue()
   {
      return taskQueue;
   }

   public static ArrayDeque<actionTypes> getKeyQueue()
   {
      return KeyQueue;
   }

   public static ArrayList<BehaviorAction> getActionsBehavior()
   {
      Collections.reverse(actionsBehavior);
      return actionsBehavior;
   }

   public static ArrayList<BehaviorBuilder.actionTypes> getActionTypes()
   {
      return actionsType;
   }


   public static ArrayList<List<actionTypes>> getActionsTypeList()
   {
      return actionsTypeList;
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
         if (listforFlags.get(i).equals(actionTypes.Pelvis))
         {
            flags.add(i, actionTypes.Pelvis);
         }

         if (listforFlags.get(i).equals(actionTypes.LeftArm))
         {
            flags.add(i, actionTypes.LeftArm);
         }

         if (listforFlags.get(i).equals(actionTypes.RightArm))
         {
            flags.add(i, actionTypes.RightArm);
         }
         if (listforFlags.get(i).equals(actionTypes.LeftLeg))
         {
            flags.add(i, actionTypes.LeftLeg);
         }
         if (listforFlags.get(i).equals(actionTypes.RightLeg))
         {
            flags.add(i, actionTypes.RightLeg);
         }
      }

      return flags;
   }


   public static void methodcollection()
   {

   }
   public void checkJointTrajectoryMessage(Subscriber<JointspaceTrajectoryStatusMessage> message)
   {
      JointspaceTrajectoryStatusMessage tmp = message.takeNextData();

      if (tmp.getTrajectoryExecutionStatus() == JointspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_STARTED);

   }

   public void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
      WalkingStatusMessage tmp = message.takeNextData();

      if (tmp.getWalkingStatus() == WalkingStatusMessage.COMPLETED);
   }

   public void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {

      TaskspaceTrajectoryStatusMessage tmp = message.takeNextData();

      if (tmp.getTrajectoryExecutionStatus() == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
         ;
   }

   public static void main(String[] args)
   {
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

//
      BehaviorBuilder build1 = new BehaviorBuilder(action1, actionTypes.Pelvis);// , ros2Node, robotModel);
      BehaviorBuilder build2 = new BehaviorBuilder(action2,actionTypes.LeftArm);//, ros2Node, robotModel) ;
      BehaviorBuilder build3 = new BehaviorBuilder(action1, actionTypes.RightLeg,actionTypes.RightArm);



//      submitTaskForParallelExecution(action1, actionTypes.LeftArm);
//      submitSingleTask(action2);
//      submitTaskForParallelExecution(action2, actionTypes.Chest);
//      submitSingleTask(action1, actionTypes.LeftArm);
//
//      System.out.println(getTaskQueue());
//      System.out.println(getKeyQueue());
//      abc(getActionTypes());
//
//      buildActionsTypeList(2,actionTypes.RightArm, actionTypes.RightLeg);
//      buildActionsTypeList(1,actionTypes.Pelvis);
//      System.out.println(getActionTypes());
      System.out.println(getActionsBehavior());
      System.out.println(getActionsTypeList());
      System.out.println(getActionsTypeList().get(1).size());

      //
//      BehaviorBuilder build3 = new BehaviorBuilder(actionTypes.Chest,action1);
//      BehaviorBuilder build4 = new BehaviorBuilder(actionTypes.Pelvis,action2);
//      BehaviorBuilder build6 = new BehaviorBuilder(actionTypes.Pelvis,action2);
//      BehaviorBuilder build5 = new BehaviorBuilder( actionTypes.Pelvis,action2);
//      ArrayList<Integer> flug =  BehaviorBuilder.buildFlags();
//      ArrayList<BehaviorBuilder.actionTypes> flug =  BehaviorBuilder.buildFlags();
//      System.out.println(BehaviorBuilder.getActionTypes());
//
//      for(int i = 0; i< getActionTypes().size(); i++)
//      {
//         if(getActionTypes().get(i).equals("Pelvis"))
//         {
//            System.out.println("Pelvis Match - should be 1 times");
//         }
//      }
//      System.out.println(BehaviorBuilder.getActionsBehavior());



   }
}
