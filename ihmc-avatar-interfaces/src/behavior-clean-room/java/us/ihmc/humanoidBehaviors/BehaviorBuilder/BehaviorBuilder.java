package us.ihmc.humanoidBehaviors.BehaviorBuilder;

import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
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


   private static int typeCounter = 0;
   private int behaviorCounter = 0;
   private static ArrayList<BehaviorBuilder.actionTypes> actionsType = new ArrayList<>(100);
   private static ArrayList<List<actionTypes>> actionsTypeList = new ArrayList<>();
   private static ArrayList<List<actionTypes>> finaList = new ArrayList<>();
   private Ros2Node ros2node;
   private DRCRobotModel robotModel;


   private static ArrayList<BehaviorAction> actionsBehavior = new ArrayList<>(100);
   private static ArrayList<BehaviorBuilder.actionTypes> flags = new ArrayList<>();


   private BehaviorBuilder.actionTypes  type;

   private static final ArrayDeque<BehaviorAction> taskQueue = new ArrayDeque<>();


   public BehaviorBuilder(BehaviorAction behaviorAction,actionTypes... type)
   {
      this(null, null, behaviorAction, type);
   }

   public BehaviorBuilder(Ros2Node ros2Node, DRCRobotModel robotModel, BehaviorAction behaviorAction, actionTypes... type)
   {
      this.ros2node = ros2Node;
      this.robotModel = robotModel;

      for(int i = 0; i< type.length ; i++)
      {
         this.type = type[i];
      }

      buildActionsTypeList(type.length , type);
      helperMethodforactions(behaviorAction);

   }


   private void helperMethodforactions(BehaviorAction action)
   {
      actionsBehavior.add(behaviorCounter, action);
      behaviorCounter++;
   }

   private static void buildActionsTypeList(int number, actionTypes... actionsType)
   {
      ArrayList<actionTypes> tmp = new ArrayList<>();

      for(int i = 0 ; i < number; ++i)
      {
         tmp.add(actionsType[i]);
      }
      actionsTypeList.add(tmp);
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

   public static ArrayList<BehaviorBuilder.actionTypes> buildFlags()
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

      BehaviorBuilder build1 = new BehaviorBuilder(action1, actionTypes.Pelvis);// , ros2Node, robotModel);
      BehaviorBuilder build2 = new BehaviorBuilder(action2,actionTypes.LeftArm);//, ros2Node, robotModel) ;
      BehaviorBuilder build3 = new BehaviorBuilder(action1, actionTypes.RightLeg,actionTypes.RightArm);

      System.out.println(getActionsBehavior());
      System.out.println(getActionsTypeList());
      System.out.println(getActionsTypeList().get(1).size());




   }
}
