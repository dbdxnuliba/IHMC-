package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple2D.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.*;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchAndKickBehavior.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.humanoidBehaviors.communication.*;
import us.ihmc.humanoidBehaviors.stateMachine.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.stateMachine.factories.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.wholeBodyController.*;
import us.ihmc.yoVariables.variable.*;

//wrtie a code to kick things
public class SearchAndKickBehavior extends StateMachineBehavior<WalkThroughDoorWOFiducialStates>
{
   private final boolean DEBUG = false;
   private boolean isDoorOpen = false;
   private static Point2D offsetFromSphere;

   //create a list of states
   public enum WalkThroughDoorWOFiducialStates
   {
      SEARCH_FOR_SPHERE, //start the action
      SETUP_ROBOT, //set up robot
      WALK_TO_THE_OBJECT, //walk till the object
      KICK_ACTION, //kicking action
      RESET_ROBOT, //reset to original configuration
      FAILED, //did it fail or not
      DONE //done state
      //add a stop behavior (checkout behavior dispatcher line179 more details)
   }



   //set waypoint for walk to interactable objects w.r.t to the object location

   private Vector3D offsetwaypoint1 = new Vector3D(0.5f,-0.9f,0f); //these indicate that you are kinda swaying but why???
   private Vector3D offsetwaypoint2 = new Vector3D(0.5f,-0.6f,0f);

   private boolean BALL_DETECTION = true;

   private final double standingDistance = 0.4;

//   private final PipeLine<AbstractBehavior> pipeLine;
   //define the behavior that will be required
//   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final SleepBehavior sleepBehavior;
   private final StepUpDoor environment;
   private YoBoolean yoDoubleSuport;
   private Pose3D doorPose = new Pose3D();

   private final ResetRobotBehavior resetRobotBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final KickBehavior kickTheBall;
//   private final SphereDetectionBehavior sphereDetctionBehavior;
   private final SearchForDoorBehavior doorLocationBehavior;

   private final HumanoidReferenceFrames referenceFrames;

//   private IHMCROS2Publisher<ValveLocationPacket> xposoffsetpublisher;

   private final ConcurrentListeningQueue<ValveLocationPacket> sphereLocation = new ConcurrentListeningQueue<>(10);


   // create a constructor
   public SearchAndKickBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                FullHumanoidRobotModel fullHumanoidRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters,
                                YoBoolean yoDoubleSupport)
   {
      this(robotName,ros2Node,yoTime,referenceFrames,fullHumanoidRobotModel,wholeBodyControllerParameters,yoDoubleSupport,null);
   }


   public SearchAndKickBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                FullHumanoidRobotModel fullHumanoidRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters,
                                YoBoolean yoDoubleSupport, StepUpDoor environment)
   {
      super(robotName, "SearchAndKickBehavior",WalkThroughDoorWOFiducialStates.class,yoTime, ros2Node);
      this.referenceFrames = referenceFrames;
//      pipeLine = new PipeLine<>(yoTime);

      sleepBehavior = new SleepBehavior(robotName,ros2Node, yoTime);
      kickTheBall = new KickBehavior(robotName,ros2Node,yoTime,yoDoubleSupport, fullHumanoidRobotModel,referenceFrames);
      this.environment = environment;
      this.yoDoubleSuport = yoDoubleSupport;

      walkToLocationBehavior = new WalkToLocationBehavior(robotName,ros2Node, fullHumanoidRobotModel,referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
//      sphereDetctionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      doorLocationBehavior = new SearchForDoorBehavior(robotName,"DoorBehavior", ros2Node,null);

      setupStateMachine();
   }



   @Override
   public void doControl()
   {
//      xposoffsetpublisher = new IHMCROS2Publisher<>(ros2Node,ValveLocationPacket.class);
//      if(sphereLocation.isNewPacketAvailable())
//      {
//         receivedSphereLocation(sphereLocation.getLatestPacket());
//      }
      super.doControl();
   }

//   public void receivedSphereLocation(ValveLocationPacket sphereLocationPacket)
//   {
//      setSphereLocation(sphereLocationPacket.getValvePoseInWorld());
//   }

//   public void setSphereLocation(Pose3D spherePosePacket)
//   {
//      spherePose = spherePosePacket;
//   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("kicking ball behavior");
      super.onBehaviorEntered();
   }

   @Override
   public WalkThroughDoorWOFiducialStates configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkThroughDoorWOFiducialStates, BehaviorAction> factory)
   {
      // initialize behaviors with their respective onBehaviorEntered() and isDone and add them to the factory

      //reset the robot
      BehaviorAction resetrobot = new BehaviorAction(resetRobotBehavior);

      BehaviorAction findBall = null;

      if(BALL_DETECTION)
      {
         //find the ball

         findBall = new BehaviorAction(doorLocationBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               publishTextToSpeech("Entering sphere detection behavior");
            }
         };
      }

      // walk towards the object
      BehaviorAction walktowardstheObject = new BehaviorAction(walkToLocationBehavior)
      {
         @Override protected void setBehaviorInput ()
         {

            publishTextToSpeech("Entering walkToInteractableObjectBehavior");
            walkToLocationBehavior.setTarget(getoffsetPoint());
         }


      };
      // get in position and kick the object

      BehaviorAction kick = new BehaviorAction(kickTheBall)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Kicking the ball");
            FramePoint2D balltoKickLocation = new FramePoint2D(getoffsetPoint().getPosition());
            kickTheBall.setObjectToKickPoint(balltoKickLocation);
         }


      };

      BehaviorAction doneState = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            sleepBehavior.setSleepTime(1);
            publishTextToSpeech("Finished kicking the object");
         }
      };

      if(BALL_DETECTION)
      {
         //start adding them to the state factory
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.SETUP_ROBOT,resetrobot,WalkThroughDoorWOFiducialStates.SEARCH_FOR_SPHERE);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.SEARCH_FOR_SPHERE,findBall,WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT,walktowardstheObject,WalkThroughDoorWOFiducialStates.KICK_ACTION);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.KICK_ACTION,kick,WalkThroughDoorWOFiducialStates.DONE);
         factory.addState(WalkThroughDoorWOFiducialStates.DONE,doneState);
      }

      else
      {
         //start adding them to the state factory
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.SETUP_ROBOT,resetrobot,WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT,walktowardstheObject,WalkThroughDoorWOFiducialStates.KICK_ACTION);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.KICK_ACTION,kick,WalkThroughDoorWOFiducialStates.DONE);
         factory.addState(WalkThroughDoorWOFiducialStates.DONE,doneState);
      }

      //return the initial key
      return WalkThroughDoorWOFiducialStates.SETUP_ROBOT;
   }


   private FramePose2D getoffsetPoint() //this is my d
   {
      FramePoint2D ballPosition2d;
      if(!BALL_DETECTION)
      {
          ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), environment.getInitialSpherePosition(),
                                                                   0.0); //this is the ball position
      }
//      FramePoint2D ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), environment.getInitialSpherePosition(),
//                                                     0.0); //this is the ball position
      else
      {
         ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(),doorLocationBehavior.getLocation().getPosition().getX(),
                                                        doorLocationBehavior.getLocation().getPosition().getY());
      }

      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(), 0.0, 0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(referenceFrames.getWorldFrame());
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      System.out.println(walkingYaw);
//      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance; //
//      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance; // these two are our distance vector
      double x;
      double y;
      if(ContactableSphereRobot.getDefaultRadius() >= 0.5)
      {
         x = ballPosition2d.getX() - (ContactableSphereRobot.getDefaultRadius() + 0.25);
         y = ballPosition2d.getY() - (0.25*Math.tan(walkingYaw));
      }
      else if (ContactableSphereRobot.getDefaultRadius() >= 0.25)
      {
         x = ballPosition2d.getX() - (ContactableSphereRobot.getDefaultRadius() + 0.15);
         y = ballPosition2d.getY() - (0.15*Math.tan(walkingYaw));
      }

      else
      {
         x = ballPosition2d.getX() - (ContactableSphereRobot.getDefaultRadius() + 0.1);
         y = ballPosition2d.getY() - (0.1*Math.tan(walkingYaw));
      }

      offsetFromSphere = new Point2D(x,y);
      //xposoffsetpublisher.publish(HumanoidMessageTools.createValveLocationPacket(true,true));

      FramePose2D poseToWalkTo = new FramePose2D(referenceFrames.getWorldFrame(), offsetFromSphere, walkingYaw);
      return poseToWalkTo;
   }
   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Leaving this behavior");
   }

   public static Point2D getOffsetFromSphere()
   {
      return offsetFromSphere;
   }

   public void setBALL_DETECTION(boolean BALL_DETECTION)
   {
      this.BALL_DETECTION = BALL_DETECTION;
   }

   public boolean getBALL_DETECTION()
   {
      return BALL_DETECTION;
   }
}
