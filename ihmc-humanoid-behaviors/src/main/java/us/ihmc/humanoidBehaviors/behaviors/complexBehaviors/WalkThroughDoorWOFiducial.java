package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import com.sun.xml.internal.bind.v2.*;
import controller_msgs.msg.dds.*;
import us.ihmc.communication.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.*;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorWOFiducial.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.*;
import us.ihmc.humanoidBehaviors.stateMachine.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.robotics.stateMachine.factories.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.yoVariables.variable.*;

//wrtie a code to kick things
public class WalkThroughDoorWOFiducial extends StateMachineBehavior<WalkThroughDoorWOFiducialStates>
{
   private final boolean DEBUG = false;
   private boolean isDoorOpen = false;

   //create a list of states
   public enum WalkThroughDoorWOFiducialStates
   {
      START, //start the action
      SETUP_ROBOT, //set up robot

      RESET_ROBOT, //reset to original configuration
      FAILED, //did it fail or not
      DONE //done state
   }

   //reset your arms

   private final boolean TUCK_IN_ARMS = true;

   //set waypoint for walkto interactableobjects w.r.t to the doorlocation receved

   private Vector3D offsetwaypoint1 = new Vector3D(0.5f,-0.9f,0f); //these indicate that you are kinda swaying but why???
   private Vector3D offsetwaypoint2 = new Vector3D(0.5f,-0.6f,0f);

   //define the behavior that will be required
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final SleepBehavior sleepBehavior;

   //environment variable
   DefaultCommonAvatarEnvironment environment;

   //TODO include a new behavior that extracts door location directly from the environemnt and supplied to an exisiting StateMachineFactory
   private final OpenDoorBehavior openDoorBehavior;
   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;
   private final ResetRobotBehavior resetRobotBehavior;

   private DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;

   //create publishers and subscribers if any
   private IHMCROS2Publisher<DoorLocationPacket> doortobehaviorpublisher;
   private IHMCROS2Publisher<DoorLocationPacket> doortoUIpublisher;
   private final HumanoidReferenceFrames referenceFrames;


   // create a constructor
   public WalkThroughDoorWOFiducial(String robotName, String yoNamePrefix, Ros2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                    AtlasPrimitiveActions atlasPrimitiveActions, YoGraphicsListRegistry yoGraphicsListRegistry, DefaultCommonAvatarEnvironment environment)
   {
      super(robotName, "WalkThroughDoorWOFiducial",WalkThroughDoorWOFiducialStates.class,yoTime, ros2Node);
      this.referenceFrames = referenceFrames;

      //TODO probobably write a modified doorOpenDetctorService independent of fiducial detection
      doorOpenDetectorBehaviorService = new DoorOpenDetectorBehaviorService(robotName, yoNamePrefix + "DoorOpenService",ros2Node, yoGraphicsListRegistry);
      registry.addChild(doorOpenDetectorBehaviorService.getYoVariableRegistry());
      addBehaviorService(doorOpenDetectorBehaviorService);

      sleepBehavior = new SleepBehavior(robotName,ros2Node, yoTime);

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //setting up environment variable
      this.environment = environment;
      //setting up behaviors

      openDoorBehavior = new OpenDoorBehavior(robotName,"OpenDoor",yoTime,ros2Node,atlasPrimitiveActions,doorOpenDetectorBehaviorService,
                                              yoGraphicsListRegistry);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(robotName,yoTime,ros2Node, atlasPrimitiveActions);
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);

      //setting up publisher
      doortobehaviorpublisher = createBehaviorOutputPublisher(DoorLocationPacket.class);
      doortoUIpublisher = createBehaviorInputPublisher(DoorLocationPacket.class);

      //calling parent setup method which configures the stateMachineFactory()
      setupStateMachine();
   }



   //doControl
   @Override
   public void doControl()
   {
      // should continuously check if the door is open or not
      //Todo replace this with the new behavior service independent of the fiducial detection
      if(doorOpenDetectorBehaviorService.newPose != null)
      {
         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         doorOpenDetectorBehaviorService.newPose.get(location,orientation);
         publishUIPositionCheckerPacket(location, orientation); //dont think this is necessary for now
      }

      if(isDoorOpen != doorOpenDetectorBehaviorService.isDoorOpen())
      {
         isDoorOpen = true; // make it more robust by writing = doorOpen##.isdone() which will return true
         publishTextToSpeech(" Door is open");
      }


      //access the door pose
      //TODO : Is there a better way to do this ( probably do this in the new behavior you write)
      StepUpDoor tmp = new StepUpDoor(0.5,1.7,0.3);
      Pose3D doorFramePose = new Pose3D(tmp.getDoorFramePose().getPosition(), tmp.getDoorFramePose().getOrientation());

      //over write the new pose3d with a door pose abstracted from the environment
      doortobehaviorpublisher.publish(HumanoidMessageTools.createDoorLocationPacket(doorFramePose));
      doortoUIpublisher.publish(HumanoidMessageTools.createDoorLocationPacket(doorFramePose));
   }

   //OnBehavviorEntered
   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Starting door detection and walking through door behavior in absence of fiducial");
      super.onBehaviorEntered();
   }
   //ConfigureStateMAchineANdInitialKEy
   @Override
   public WalkThroughDoorWOFiducialStates configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkThroughDoorWOFiducialStates, BehaviorAction> factory)
   {
         // initialize behaviors with their respective onBehaviorEntered() and isDone and add them to the factory

      //reset the robot

      //start and put the arms in desired position

      // start searching for the door

      // walk towards the door

      // start door opening behavior

      //set up and walk through door

      //what to do in case of failure behavior

      //add done state

      //start adding them to the state factory


      //return the initial key
      return WalkThroughDoorWOFiducialStates.START;
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Leaving this behavior");
   }
}
