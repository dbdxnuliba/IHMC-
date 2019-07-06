package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.robotics.taskExecutor.*;
import us.ihmc.ros2.*;
import us.ihmc.yoVariables.variable.*;

public class KickTheBall extends AbstractBehavior
{
   private final GoHomeBehavior chestGoHomeBehavior;
   private final GoHomeBehavior pelvisGoHomeBehavior;
   private final GoHomeBehavior armGoHomeLeftBehavior;
   private final GoHomeBehavior armGoHomeRightBehavior;

   boolean leftArm = true;
   boolean rightArm = true;
   boolean chest = true;
   boolean pelvis = true;


   private final PipeLine<AbstractBehavior> pipeLine;

   //write constructor
   public KickTheBall(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, ros2Node, yoTime,true, true, true, true);
   }

   public KickTheBall(String robotName, Ros2Node ros2Node, YoDouble yoTime, boolean leftArm, boolean rightArm, boolean chest, boolean pelvis)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);

      chestGoHomeBehavior = new GoHomeBehavior(robotName, "chest", ros2Node, yoTime);

      pelvisGoHomeBehavior = new GoHomeBehavior(robotName, "pelvis", ros2Node, yoTime);

      armGoHomeLeftBehavior = new GoHomeBehavior(robotName, "leftArm", ros2Node, yoTime);
      armGoHomeRightBehavior = new GoHomeBehavior(robotName, "rightArm", ros2Node, yoTime);
   }

   @Override
   public void doControl()
   {
      if (!isPaused())
      {
         pipeLine.doControl();
      }
   }

   private void setupPipeLine()
   {

   }
   @Override
   public void onBehaviorEntered()
   {

   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public void onBehaviorExited()
   {

   }



   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}
