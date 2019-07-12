package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.awt.geom.*;
import java.lang.reflect.*;
import java.util.ArrayList;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootLoadBearingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.*;
import us.ihmc.humanoidBehaviors.taskExecutor.FootTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class KickBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble yoTime;
   private final ReferenceFrame midZupFrame;
   private YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet", registry); //the registry is similar to behavior name
   private final FootTrajectoryBehavior footTrajectoryBehavior;
   private YoBoolean yoprint;

   private FramePoint2D objectToKickPose;

   private final ArrayList<AbstractBehavior> behaviors = new ArrayList<AbstractBehavior>();

   private final PipeLine<AbstractBehavior> pipeLine;
   private final YoDouble trajectoryTime;
   private final SideDependentList<MovingReferenceFrame> ankleZUpFrames;
   protected  final ConcurrentListeningQueue<LocalizationPacket> spherexlocation = new ConcurrentListeningQueue<>(10);

   //create an array of double of x,y,z
  private double pos[][] = {{0.0,0.25,0.277},{-0.2,0.15,0.127,},{0.3,0.15,0.05},{0.0,0.25,0.127},{0.0,0.25,0.0}};

   public KickBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                       FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.yoTime = yoTime;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();
      trajectoryTime = new YoDouble("kickTrajectoryTime", registry);
      trajectoryTime.set(0.5);
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      footTrajectoryBehavior = new FootTrajectoryBehavior(robotName, ros2Node, yoTime, yoDoubleSupport); //this guy also takes in yoDoubleSupport and the yoDoubleSupport is getting updated
      registry.addChild(footTrajectoryBehavior.getYoVariableRegistry());
      this.yoprint = yoDoubleSupport;
      createBehaviorInputSubscriber(LocalizationPacket.class,spherexlocation::put);

   }

   @Override
   public void doControl()
   {
      if (!hasInputBeenSet.getBooleanValue())
      {
         // setObjectToKickPoint(new
         // FramePoint(ReferenceFrame.getWorldFrame(),0,0,0));
         checkInput();
      }
      else
      {
         pipeLine.doControl();
         int t = 0;
         if(t%10000 == 0)
         {
            //System.out.println(spherexlocation.getLatestPacket().getPoint2DX());
            t++;
         }
      }
   }

   public void setObjectToKickPoint(FramePoint2D objectToKickPose)
   {
      this.objectToKickPose = objectToKickPose;
   }

   private void checkInput()
   {
      if (objectToKickPose != null)
      {
         hasInputBeenSet.set(true);
         setupPipeline();
      }
   }

   private void setupPipeline()
   {
      final RobotSide kickFoot = RobotSide.LEFT; //left foot will be used to emulate kick behavior

      // submitFootPosition(kickFoot.getOppositeSide(), new
      // FramePoint(ankleZUpFrames.get(kickFoot), 0.0,
      // kickFoot.getOppositeSide().negateIfRightSide(0.25), 0.227));
      //
      // submitFootPosition(kickFoot.getOppositeSide(), new
      // FramePoint(ankleZUpFrames.get(kickFoot), 0.0,
      // kickFoot.getOppositeSide().negateIfRightSide(0.35), 0.227));
      //
      //
      //
      // submitFootPosition(kickFoot.getOppositeSide(), new
      // FramePoint(ankleZUpFrames.get(kickFoot), 0.0,
      // kickFoot.getOppositeSide().negateIfRightSide(0.35), 0));

      //this current behavior tends to hit things in net downwards direction.


      for(int i=0; i<pos.length; i++)
      {
         for(int j=0; j<pos[i].length ; j++)
         {
            //set the x value here
            if(j ==0 && i ==3)
            {
               if (ContactableSphereRobot.getDefaultRadius() > 0.5)
               {
                  pos[i][j] = 0.25 + ContactableSphereRobot.getDefaultRadius();
               }
               else if(ContactableSphereRobot.getDefaultRadius() <= 0.5 && ContactableSphereRobot.getDefaultRadius() >=0.25)
               {
                  pos[i][j] = 0.15 + ContactableSphereRobot.getDefaultRadius();
               }
               else
               {
                  pos[i][j] = 0.1 + ContactableSphereRobot.getDefaultRadius();
               }
            }
            else if(j ==1 && i ==3)
            {
               if(ContactableSphereRobot.getDefaultRadius() <0.15)
               {
                  pos[i][j] = 0.08;
               }

            }

         }
      }

      for(int i = 0 ; i < pos.length; i++)
      {
         submitFootPosition(kickFoot,new FramePoint3D(ankleZUpFrames.get(kickFoot.getOppositeSide()),pos[i]));
      }



      //      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.0, kickFoot.negateIfRightSide(0.25), 0.227)); //negates the value itself
//
//      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrames.get(kickFoot.getOppositeSide()), -0.2, kickFoot.negateIfRightSide(0.15), 0.127));
//
//      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrames.get(kicpollkFoot.getOppositeSide()), 0.3, kickFoot.negateIfRightSide(0.15), 0.05));
//
//      // submitFootPosition(kickFoot, new
//      // FramePoint(objectToKickPose.getReferenceFrame(),
//      // objectToKickPose.getX(), objectToKickPose.getY(), 0.127));
//
//      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.0, kickFoot.negateIfRightSide(0.25), 0.127));
//
//      submitFootPosition(kickFoot, new FramePoint3D(ankleZUpFrames.get(kickFoot.getOppositeSide()), 0.0, kickFoot.negateIfRightSide(0.25), 0));

      final FootLoadBearingBehavior footStateBehavior = new FootLoadBearingBehavior(robotName, ros2Node);
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

   private void submitFootPosition(RobotSide robotSide, FramePoint3D desiredFootPosition)
   {
      FrameQuaternion desiredFootOrientation = new FrameQuaternion(desiredFootPosition.getReferenceFrame());
      FramePose3D desiredFootPose = new FramePose3D(desiredFootPosition, desiredFootOrientation);
      submitFootPose(robotSide, desiredFootPose);
   }

   private void submitFootPose(RobotSide robotSide, FramePose3D desiredFootPose)
   {
      desiredFootPose.changeFrame(worldFrame);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      desiredFootPose.get(desiredFootPosition, desiredFootOrientation); //assign the value from wroldframe perspective the position and orientation value
      FootTrajectoryTask task = new FootTrajectoryTask(robotSide, desiredFootPosition, desiredFootOrientation, footTrajectoryBehavior,
                                                       trajectoryTime.getDoubleValue());
      pipeLine.submitSingleTaskStage(task);
   }

   @Override
   public void onBehaviorEntered()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorEntered();
      }
   }

   @Override
   public void onBehaviorExited()
   {
      hasInputBeenSet.set(false);
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorExited();
      }
   }

   @Override
   public void onBehaviorAborted()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorAborted();
      }
   }

   @Override
   public void onBehaviorPaused()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorPaused();
      }
   }

   @Override
   public boolean isDone()
   {
      return hasInputBeenSet() && pipeLine.isDone();
   }

   @Override
   public void onBehaviorResumed()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorResumed();
      }
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
