package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.*;
import javafx.geometry.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.humanoidBehaviors.behaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.humanoidBehaviors.taskExecutor.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.taskExecutor.*;
import us.ihmc.ros2.*;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.awt.geom.*;
import java.util.*;

public class DynamicKick extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine;
   private YoDouble yoTime;
   private YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet",registry);
   private final FootTrajectoryBehavior footTrajectoryBehavior;
   private final ArrayList<AbstractBehavior> behaviors = new ArrayList<>();
   private YoBoolean yoDoubleSupport;
   private SideDependentList<MovingReferenceFrame> ankleZUpFrame;
   private PelvisTrajectoryBehavior pelvisTrajectoryBehavior;
   private ArmTrajectoryBehavior armTrajectoryBehavior;
   private ChestTrajectoryBehavior chestTrajectoryBehavior;
   private FramePoint2D objectToKickPose;
   private final YoDouble trajectoryTime;
   private HumanoidReferenceFrames referenceFrames;
   private double counter = 1.0;
   private double tuningParam = 1.5;

//   private FootTrajectoryTask[] tasks = new FootTrajectoryTask[6];


   public DynamicKick(String robotName, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                      HumanoidReferenceFrames referenceFrames)
   {
      super(robotName,ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.yoTime = yoTime;
      footTrajectoryBehavior = new FootTrajectoryBehavior(robotName,ros2Node,yoTime,yoDoubleSupport);
      trajectoryTime = new YoDouble("kickTrajectoryTime",registry);
      trajectoryTime.set(tuningParam/2);
      this.yoDoubleSupport = yoDoubleSupport;
      this.ankleZUpFrame = referenceFrames.getAnkleZUpReferenceFrames();
      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(robotName,ros2Node,yoTime);
      armTrajectoryBehavior = new ArmTrajectoryBehavior(robotName,ros2Node,yoTime);
      chestTrajectoryBehavior = new ChestTrajectoryBehavior(robotName, ros2Node,yoTime);
      this.referenceFrames = referenceFrames;

   }

   @Override
   public void doControl()
   {
      if(!hasInputBeenSet.getBooleanValue())
      {
         checkInput();
      }
      else
      {
         pipeLine.doControl();
      }
   }

   public void setObjectToKickPose(FramePoint2D objectToKickPose)
   {
      this.objectToKickPose = objectToKickPose;
   }

   private void checkInput()
   {
      if(objectToKickPose != null)
      {
         hasInputBeenSet.set(true);
         setUpPipeline();
      }
   }

   private void setUpPipeline()
   {

      //adding all the footstep trajectories into an array sending this independently
      final RobotSide kickFoot = RobotSide.RIGHT;

      //take arms slightly backwards - taken from TheFinalTab class
      final double armTrajectorTime = 0.5;
      //      double[] jointAngles = new double[] {0.765356719493866, 0.024195531383156776, 2.9822821617126465, 1.6808037757873535, -0.3247416913509369, 0.67205411195755, 0.15090779960155487};
      double[] jointAngles = new double[] {Math.toRadians(15.0),Math.toRadians(-90.0),Math.toRadians(90.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0),Math.toRadians(0.0)};
      //      double[] jointAngles = new double[] {0,Math.toRadians(-90.0),Math.toRadians(75), Math.toRadians(60.0),0,0,0};
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, armTrajectorTime, jointAngles);


      ArmTrajectoryTask leftArmTask = new ArmTrajectoryTask(armTrajectoryMessage,armTrajectoryBehavior);
      pipeLine.submitSingleTaskStage(leftArmTask);

      //      jointAngles = new double[] {0.10722935199737549, -0.23587453365325928, 2.419130802154541, -0.9118338823318481, -2.2621233463287354, -0.5176281929016113, 0.005108347628265619};
      jointAngles = new double[] {0.0,Math.toRadians(90.0),0.0,0.0,0.0,0.0,0.0};
      //      jointAngles = new double[] {0,Math.toRadians(90.0),Math.toRadians(75), Math.toRadians(-60.0),0,0,0};
      armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, armTrajectorTime, jointAngles);

      ArmTrajectoryTask rightArmTask = new ArmTrajectoryTask(armTrajectoryMessage,armTrajectoryBehavior);
      pipeLine.submitSingleTaskStage(rightArmTask);


      //Get Pelvis up
      MovingReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FramePoint3D pelvisReference = new FramePoint3D(pelvisZUpFrame);
      pelvisReference.changeFrame(referenceFrames.getWorldFrame());

      final double pelvisTrajectoryTime = 0.5;
      Pose3D desiredpelvisPose = new Pose3D(pelvisReference.getX(),pelvisReference.getY() + 0.1,pelvisReference.getZ(),0.0,0.0,0.0);
      PelvisTrajectoryMessage pelvismesssage = HumanoidMessageTools.createPelvisTrajectoryMessage(pelvisTrajectoryTime,desiredpelvisPose);
      pelvismesssage.enable_user_pelvis_control_= true;
      pelvismesssage.enable_user_pelvis_control_during_walking_ = true;
      PelvisTrajectoryTask pelvisTask = new PelvisTrajectoryTask(pelvismesssage,pelvisTrajectoryBehavior);
      pipeLine.submitSingleTaskStage(pelvisTask);




      //take leg up c = 1
      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrame.get(kickFoot.getOppositeSide()),0.0,-0.25,0.227));


      //bend torso forward and also take pelvisx a bit back
      final double chestTrajectoryTime = tuningParam;
      FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      chestOrientation.setYawPitchRollIncludingFrame(referenceFrames.getWorldFrame(),0.0, Math.toRadians(30.0), 0.0);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(chestTrajectoryTime, chestOrientation,referenceFrames.getWorldFrame(),referenceFrames.getPelvisZUpFrame());
      ChestOrientationTask chestTask = new ChestOrientationTask(chestTrajectoryMessage,chestTrajectoryBehavior);
//      pipeLine.submitSingleTaskStage(chestTask);
      pipeLine.submitTaskForPallelPipesStage(chestTrajectoryBehavior,chestTask);

      //Take right leg backwards c = 2
      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrame.get(kickFoot.getOppositeSide()),-0.2,-0.15,0.127));


      //taking pelvisx back
//      Pose3D pelvisBack = new Pose3D(desiredpelvisPose.getPosition().getX() - 0.05,desiredpelvisPose.getPosition().getY(),desiredpelvisPose.getPosition().getZ(),0.0,0.0,0.0);
//      pelvismesssage = HumanoidMessageTools.createPelvisTrajectoryMessage(pelvisTrajectoryTime,pelvisBack);
//      pelvisTask = new PelvisTrajectoryTask(pelvismesssage,pelvisTrajectoryBehavior);
//      pipeLine.submitSingleTaskStage(pelvisTask);


      //take right leg more backwards c = 3
      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrame.get(kickFoot.getOppositeSide()),-0.35,-0.15,0.127));


      //simultaneously take both hands back too



      //simultaneously straighten torso and start hand action forward
      chestOrientation.setYawPitchRollIncludingFrame(referenceFrames.getWorldFrame(),0.0,Math.toRadians(-10),0.0);
      chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(chestTrajectoryTime,chestOrientation,referenceFrames.getWorldFrame(),referenceFrames.getPelvisZUpFrame());
      chestTask = new ChestOrientationTask(chestTrajectoryMessage,chestTrajectoryBehavior);
//      pipeLine.submitSingleTaskStage(chestTask);
      pipeLine.submitTaskForPallelPipesStage(chestTrajectoryBehavior ,chestTask);

      //get both your hands up simultaneously swiftly
      jointAngles = new double[] {0,Math.toRadians(-90.0),Math.toRadians(75), Math.toRadians(60.0),0,0,0};
      armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, tuningParam/2.5, jointAngles);
      leftArmTask = new ArmTrajectoryTask(armTrajectoryMessage,armTrajectoryBehavior);
      pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehavior , leftArmTask);

      jointAngles = new double[] {0,Math.toRadians(90.0),Math.toRadians(75), Math.toRadians(-60.0),0,0,0};
      armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, tuningParam/2.5, jointAngles);
      rightArmTask = new ArmTrajectoryTask(armTrajectoryMessage,armTrajectoryBehavior);
      pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehavior ,rightArmTask);



      //after 1 sec gap move right leg forward c = 4, 5
      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrame.get(kickFoot.getOppositeSide()),0.3,-0.15,0.1));
      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrame.get(kickFoot.getOppositeSide()),0.3,-0.25,0.0));


      //while the above two action happen move right hand to till hip position and let left hand rotate more upwards

      chestOrientation.setYawPitchRollIncludingFrame(referenceFrames.getWorldFrame(),0.0,Math.toRadians(0.0),0.0);
      chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(chestTrajectoryTime,chestOrientation,referenceFrames.getWorldFrame(),referenceFrames.getPelvisZUpFrame());
      chestTask = new ChestOrientationTask(chestTrajectoryMessage,chestTrajectoryBehavior);
      pipeLine.submitTaskForPallelPipesStage(chestTrajectoryBehavior,chestTask);
      //all things to normal c = 6
      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrame.get(kickFoot.getOppositeSide()),0.0,-0.25,0.0));


      final FootLoadBearingBehavior footStateBehavior = new FootLoadBearingBehavior(robotName,ros2Node);
      pipeLine.submitSingleTaskStage(new BehaviorAction(footStateBehavior)
       {
          @Override
          protected void setBehaviorInput()
          {
             FootLoadBearingMessage message = HumanoidMessageTools.createFootLoadBearingMessage(kickFoot, LoadBearingRequest.LOAD);
             footStateBehavior.setInput(message);
          }

       });
   }

   private void submitFootPosition(RobotSide robotSide , FramePoint3D desiredFootPosition)
   {

      FrameQuaternion desiredFootOrientation = new FrameQuaternion(desiredFootPosition.getReferenceFrame());
      FramePose3D desiredFootPose = new FramePose3D(desiredFootPosition,desiredFootOrientation);
      submitFootPose(robotSide, desiredFootPose);

   }

   private void submitFootPose(RobotSide side, FramePose3D desiredFootPose)
   {


      desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      us.ihmc.euclid.tuple3D.Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootQaternion = new Quaternion();
      desiredFootPose.get(desiredFootPosition,desiredFootQaternion);
      FootTrajectoryTask task = new FootTrajectoryTask(side,desiredFootPosition,desiredFootQaternion, footTrajectoryBehavior
            ,trajectoryTime.getDoubleValue());
//      for(int j = 0; j < tasks.length ; ++j)
//      {
//         tasks[j] = task;
//      }


      if(counter == 2 || counter == 3 || counter == 4 || counter == 5 || counter == 6)
      {
         pipeLine.submitTaskForPallelPipesStage(footTrajectoryBehavior, task);
      }
      else
      {
         pipeLine.submitSingleTaskStage(task);
      }
      counter = counter+1;
   }

   @Override
   public void onBehaviorEntered()
   {
      for(AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorEntered();
      }

   }

   @Override
   public void onBehaviorAborted()
   {
      for(AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorAborted();
      }
   }

   @Override
   public void onBehaviorPaused()
   {
      for(AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorPaused();
      }
   }

   @Override
   public void onBehaviorResumed()
   {
      for(AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorResumed();
      }

   }

   @Override
   public void onBehaviorExited()
   {
      hasInputBeenSet.set(false);
      for(AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorExited();
      }
   }

   @Override
   public boolean isDone()
   {

      return hasInputBeenSet() && pipeLine.isDone();
   }

   private boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }

}
