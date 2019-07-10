package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple2D.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchAndKickBehavior.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.humanoidBehaviors.stateMachine.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.stateMachine.factories.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.wholeBodyController.*;
import us.ihmc.yoVariables.variable.*;

//wrtie a code to kick things
public class SearchAndKickBehavior extends StateMachineBehavior<WalkThroughDoorWOFiducialStates>
{
   private final boolean DEBUG = false;
   private boolean isDoorOpen = false;

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

   //reset your arms

//   private final boolean TUCK_IN_ARMS = true;

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
   //environment variable
//   DefaultCommonAvatarEnvironment environment;


   //private final OpenDoorBehavior openDoorBehavior;
//   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;
   private final ResetRobotBehavior resetRobotBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final KickBehavior kickTheBall;
   private final SphereDetectionBehavior sphereDetctionBehavior;


   //create publishers and subscribers if any
   //private IHMCROS2Publisher<DoorLocationPacket> doortobehaviorpublisher;
   //private IHMCROS2Publisher<DoorLocationPacket> doortoUIpublisher;
   private final HumanoidReferenceFrames referenceFrames;


   // create a constructor
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

//      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //setting up environment variable
//      this.environment = environment;
      //setting up behaviors

//      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(robotName,yoTime,ros2Node, atlasPrimitiveActions);
      walkToLocationBehavior = new WalkToLocationBehavior(robotName,ros2Node, fullHumanoidRobotModel,referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      sphereDetctionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      //setting up publisher
      //doortobehaviorpublisher = createBehaviorOutputPublisher(DoorLocationPacket.class);
      //doortoUIpublisher = createBehaviorInputPublisher(DoorLocationPacket.class);

      //calling parent setup method which configures the stateMachineFactory()
      setupStateMachine();
   }


//   doControl
//   @Override
//   public void doControl()
//   {
//      // should continuously check if the door is open or not
//      //Todo replace this with the new behavior service independent of the fiducial detection
//      if(doorOpenDetectorBehaviorService.newPose != null)
//      {
//         Point3D location = new Point3D();
//         Quaternion orientation = new Quaternion();
//         doorOpenDetectorBehaviorService.newPose.get(location,orientation);
//         publishUIPositionCheckerPacket(location, orientation); //dont think this is necessary for now
//      }
//
//      if(isDoorOpen != doorOpenDetectorBehaviorService.isDoorOpen())
//      {
//         isDoorOpen = true; // make it more robust by writing = doorOpen##.isdone() which will return true
//         publishTextToSpeech(" Door is open");
//      }
//
//
//      //access the door pose
//      StepUpDoor tmp = new StepUpDoor(0.5,1.7,0.3);
//      Pose3D doorFramePose = new Pose3D(tmp.getDoorFramePose().getPosition(), tmp.getDoorFramePose().getOrientation());
//
//      //over write the new pose3d with a door pose abstracted from the environment
//      doortobehaviorpublisher.publish(HumanoidMessageTools.createDoorLocationPacket(doorFramePose));
//      doortoUIpublisher.publish(HumanoidMessageTools.createDoorLocationPacket(doorFramePose));
//   }

   //Also have setup my own pipeline (refer reset robot behavior)

   //OnBehavviorEntered
   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("kicking ball behavior");
      super.onBehaviorEntered();
   }

//   protected void setupPipeLine()
//   {
//      pipeLine.clearAll();
//      pipeLine.requestNewStage();
//      pipeLine.sub
//   }
   //ConfigureStateMAchineANdInitialKEy
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

         findBall = new BehaviorAction(sphereDetctionBehavior)
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

//         @Override
//         public void onExit()
//         {
//            publishTextToSpeech("exiting kick behavior");
//            super.onExit();
//            //System.out.println("sfhhf");
//         }
      };

      //BehaviorAction kickTheBall = new BehaviorAction(kicktheball);
      // get to original stance
      //BehaviorAction resetrobot2 = new BehaviorAction(resetRobotBehavior);
      //add done state
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

//   private FramePoint3D offsetPointFromTheObject(Vector3D point)
//   {
//      PoseReferenceFrame objectPose = new PoseReferenceFrame("objecPose", ReferenceFrame.getWorldFrame());
//      objectPose.setPoseAndUpdate(new Pose3D(sphereDetctionBehavior.getBallLocation()));
//
//      FramePoint3D ret = new FramePoint3D(objectPose,point);
//      return ret;
//
//   }

   private FramePose2D getoffsetPoint()
   {
      FramePoint2D ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), environment.getInitialSpherePosition()-0.2,
                                                     0.0);
//                                                     sphereDetctionBehavior.getBallLocation().getY());
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(), 0.0, 0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(referenceFrames.getWorldFrame());
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
      FramePose2D poseToWalkTo = new FramePose2D(referenceFrames.getWorldFrame(), new Point2D(x, y), walkingYaw);
      return poseToWalkTo;
   }
   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Leaving this behavior");
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
