package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple2D.*;
import us.ihmc.euclid.tuple3D.*;
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
   private final boolean DEBUG = true;
   private boolean isDoorOpen = false;
   private static Point2D offsetFromSphere;
   private static Point2D offsetFromSphereForGoalPost;

   //create a list of states
   public enum WalkThroughDoorWOFiducialStates
   {
      SEARCH_FOR_SPHERE, //start the action
      SETUP_ROBOT, //set up robot
      WALK_TO_THE_OBJECT, //walk till the object
      YAW_AS_PER_GOAL_POST, //get in position wrt to goal post
      KICK_ACTION, //kicking action
      RESET_ROBOT, //reset to original configuration
      FAILED, //did it fail or not
      DONE //done state
      //add a stop behavior (checkout behavior dispatcher line179 more details)
   }



   //set waypoint for walk to interactable objects w.r.t to the object location

   private Vector3D offsetwaypoint1 = new Vector3D(0.5f,-0.9f,0f); //these indicate that you are kinda swaying but why???
   private Vector3D offsetwaypoint2 = new Vector3D(0.5f,-0.6f,0f);

   private boolean BALL_DETECTION = false;

//   private final PipeLine<AbstractBehavior> pipeLine;
   //define the behavior that will be required
//   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final SleepBehavior sleepBehavior;
   private final StepUpDoor environment;
   private YoBoolean yoDoubleSuport;
   private Pose3D spherePose = new Pose3D();

   private final ResetRobotBehavior resetRobotBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final WalkToLocationBehavior yawAccordingToGoalPost;
   private final KickBehavior kickTheBall;
   private final SphereDetectionBehavior sphereDetctionBehavior;
   private FrameVector2D walkingDirectionForYawMotion;

   private final HumanoidReferenceFrames referenceFrames;

   private IHMCROS2Publisher<ValveLocationPacket> xposoffsetpublisher;

   private final ConcurrentListeningQueue<ValveLocationPacket> sphereLocation = new ConcurrentListeningQueue<>(10);


   // create a constructor
   public SearchAndKickBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                FullHumanoidRobotModel fullHumanoidRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters,
                                YoBoolean yoDoubleSupport)
   {
      super(robotName, "SearchAndKickBehavior",WalkThroughDoorWOFiducialStates.class,yoTime, ros2Node);
      this.referenceFrames = referenceFrames;
      //      pipeLine = new PipeLine<>(yoTime);

      sleepBehavior = new SleepBehavior(robotName,ros2Node, yoTime);
      kickTheBall = new KickBehavior(robotName,ros2Node,yoTime,yoDoubleSupport, fullHumanoidRobotModel,referenceFrames);

      this.yoDoubleSuport = yoDoubleSupport;
      environment = null;
      walkToLocationBehavior = new WalkToLocationBehavior(robotName,ros2Node, fullHumanoidRobotModel,referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      yawAccordingToGoalPost = new WalkToLocationBehavior(robotName,ros2Node,fullHumanoidRobotModel,referenceFrames,wholeBodyControllerParameters.getWalkingControllerParameters());
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      sphereDetctionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      xposoffsetpublisher = new IHMCROS2Publisher<>(ros2Node,ValveLocationPacket.class);
      createBehaviorInputSubscriber(ValveLocationPacket.class,sphereLocation::put);
      setupStateMachine();
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
      yawAccordingToGoalPost = new WalkToLocationBehavior(robotName,ros2Node,fullHumanoidRobotModel,referenceFrames,wholeBodyControllerParameters.getWalkingControllerParameters());
      walkToLocationBehavior = new WalkToLocationBehavior(robotName,ros2Node, fullHumanoidRobotModel,referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      sphereDetctionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);

      setupStateMachine();
   }



   @Override
   public void doControl()
   {
      xposoffsetpublisher = new IHMCROS2Publisher<>(ros2Node,ValveLocationPacket.class);
      if(sphereLocation.isNewPacketAvailable())
      {
         receivedSphereLocation(sphereLocation.getLatestPacket());
      }
      super.doControl();
   }

   public void receivedSphereLocation(ValveLocationPacket sphereLocationPacket)
   {
      setSphereLocation(sphereLocationPacket.getValvePoseInWorld());
   }

   public void setSphereLocation(Pose3D spherePosePacket)
   {
      spherePose = spherePosePacket;
   }

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
      // get in position by yawing the robot in accordance to the goal post orientation
      BehaviorAction yawAction = new BehaviorAction(yawAccordingToGoalPost)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Entering yaw Motion Action");
            yawAccordingToGoalPost.setTarget(getPositionwrtGoalPost());
         }
      };


      // kick the object

      BehaviorAction kick = new BehaviorAction(kickTheBall)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Kicking the ball");
            FramePoint2D balltoKickLocation = new FramePoint2D(getoffsetPoint().getPosition());
            kickTheBall.setObjectToKickPoint(balltoKickLocation); //create a new method for orienting the robot
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
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT, walktowardstheObject, WalkThroughDoorWOFiducialStates.YAW_AS_PER_GOAL_POST);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.YAW_AS_PER_GOAL_POST, yawAction,WalkThroughDoorWOFiducialStates.KICK_ACTION);
//         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT,walktowardstheObject,WalkThroughDoorWOFiducialStates.KICK_ACTION);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.KICK_ACTION,kick,WalkThroughDoorWOFiducialStates.DONE);
         factory.addState(WalkThroughDoorWOFiducialStates.DONE,doneState);
      }

      //return the initial key
      return WalkThroughDoorWOFiducialStates.SETUP_ROBOT;
   }

   private FramePose2D getPositionwrtGoalPost()
   {
      FramePoint2D goalPostPosition = new FramePoint2D(ReferenceFrame.getWorldFrame(),environment.getGoalPostPosition().getX(),environment.getGoalPostPosition().getY());
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(),0.0,0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(referenceFrames.getWorldFrame());
      walkingDirection.set(goalPostPosition);
      walkingDirection.sub(robotPosition);
      FrameVector2D tmp = getWalkingDirectionForYawMotion();
      //calcutating d value
      double distanceFromSphereBeforeYawMotion = Math.sqrt(Math.pow(tmp.getX(),2.0) + Math.pow(tmp.getY(),2.0));
//      walkingDirection.set(robotPosition);
//      walkingDirection.sub(goalPostPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      if(DEBUG)
      {
         System.out.println("goal Post Position :" + goalPostPosition.getX() + goalPostPosition.getY());
      }

      double xprime;
      double yprime;
//      xprime = offsetFromSphere.getX()+ (offsetFromSphere.getX() - (offsetFromSphere.getX()*Math.cos(walkingYaw))) + 0.1;
//      yprime = offsetFromSphere.getY() - offsetFromSphere.getY()*Math.sin(walkingYaw);
      xprime = offsetFromSphere.getX() + (distanceFromSphereBeforeYawMotion - distanceFromSphereBeforeYawMotion*Math.cos(walkingYaw));
      yprime = offsetFromSphere.getY() + (distanceFromSphereBeforeYawMotion*Math.sin(walkingYaw));

      offsetFromSphereForGoalPost = new Point2D(xprime,yprime);
      FramePose2D poseTowalkTowrtGoalPost = new FramePose2D(referenceFrames.getWorldFrame(),offsetFromSphereForGoalPost,walkingYaw);

      return poseTowalkTowrtGoalPost;
   }

   private FramePose2D getoffsetPoint() //this is my d variable
   {
      FramePoint2D ballPosition2d;
      if(!BALL_DETECTION)
      {
         ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), environment.getInitialSpherePosition(),
                                                                    0.0);
      }
      else
      {
         ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(),spherePose.getPosition().getX(),
                                                        spherePose.getPosition().getY());
      }
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(), 0.0, 0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(referenceFrames.getWorldFrame());

      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      this.walkingDirectionForYawMotion = walkingDirection;
      //walkingDirection.getX() and getY() will give you the d value
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      if(DEBUG)
      {
         System.out.println(walkingYaw);
      }
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

      offsetFromSphere = new Point2D(x,y); //offset distance d
      //xposoffsetpublisher.publish(HumanoidMessageTools.createValveLocationPacket(true,true));

      FramePose2D poseToWalkTo = new FramePose2D(referenceFrames.getWorldFrame(), offsetFromSphere, walkingYaw);
      return poseToWalkTo;
   }
   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Leaving this behavior");
   }

   public FrameVector2D getWalkingDirectionForYawMotion()
   {
      return walkingDirectionForYawMotion;
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
