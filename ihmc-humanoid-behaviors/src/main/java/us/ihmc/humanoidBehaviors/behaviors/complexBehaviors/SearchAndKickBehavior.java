package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.*;
import std_msgs.msg.dds.*;
import us.ihmc.communication.*;
import us.ihmc.communication.packets.*;
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
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.stateMachine.factories.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.wholeBodyController.*;
import us.ihmc.yoVariables.variable.*;

import javax.annotation.*;
import java.lang.*;
import java.lang.String;

//wrtie a code to kick things
public class SearchAndKickBehavior extends StateMachineBehavior<WalkThroughDoorWOFiducialStates>
{
   private final boolean DEBUG = false;
   private static Point2D offsetFromSphere;
   private static Point2D offsetFromSphereForGoalPost;

   //create a list of states
   public enum WalkThroughDoorWOFiducialStates
   {
      SEARCH_FOR_SPHERE, //start the action
      SETUP_ROBOT, //set up robot
      WALK_TO_THE_OBJECT, //walk till the object
      YAW_AS_PER_GOAL_POST, //get in position wrt to goal post
      WALK_TO_POINT_1,
      PELVIS_UP,
      KICK_ACTION, //kicking action
      PELVIS_DOWN,
      RESET_ROBOT, //reset to original configuration
      FAILED, //did it fail or not
      DONE //done state
      //add a stop behavior (checkout behavior dispatcher line179 more details)
   }

   //TODO clean up this code and ready for PR
   //TODO write a new environment made up plannar regions to test planner dependent behaviors


   //set waypoint for walk to interactable objects w.r.t to the object location

   private double distanceFromSphereBeforeYawMotion;
   private boolean BALL_DETECTION = true;

   private final SleepBehavior sleepBehavior;
   private final StepUpDoor environment;
   private YoBoolean yoDoubleSuport;
//   private Pose3D spherePose = new Pose3D();
   private AtlasPrimitiveActions atlasPrimitiveActions;
   private double walkingYawofgetoffsetPoint;

   private final ResetRobotBehavior resetRobotBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final WalkToLocationBehavior yawAccordingToGoalPost;
   private final WalkToLocationBehavior walkToOffsetPoint1;
   private final KickBehavior kickTheBall;
   private FullHumanoidRobotModel model;
   private final SearchForDoorBehavior sphereLocationDetectionBehavior;
   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;
//   private final SphereDetectionBehavior sphereDetctionBehavior;
   private FrameVector2D walkingDirectionForYawMotion;
   private boolean IS_ROBOT_AHEAD_OR_BACK;
   private static YoVariable ankletau;

   private Pose3D goalPostPose;
   private boolean pelvis_up;
   private final HumanoidReferenceFrames referenceFrames;
   private FramePose3D tmpGFP = new FramePose3D();

//   private IHMCROS2Publisher<ValveLocationPacket> xposoffsetpublisher;

   private final ConcurrentListeningQueue<ValveLocationPacket> sphereLocation = new ConcurrentListeningQueue<>(10);


   // create a constructor
   public SearchAndKickBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                FullHumanoidRobotModel fullHumanoidRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters,
                                YoBoolean yoDoubleSupport, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      this(robotName,ros2Node,yoTime,referenceFrames,fullHumanoidRobotModel,wholeBodyControllerParameters,yoDoubleSupport, atlasPrimitiveActions, null);
   }


   public SearchAndKickBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                FullHumanoidRobotModel fullHumanoidRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters,
                                YoBoolean yoDoubleSupport, AtlasPrimitiveActions atlasPrimitiveActions, StepUpDoor environment)
   {
      super(robotName, "SearchAndKickBehavior",WalkThroughDoorWOFiducialStates.class,yoTime, ros2Node);
      this.referenceFrames = referenceFrames;
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.model = fullHumanoidRobotModel;
      sleepBehavior = new SleepBehavior(robotName,ros2Node, yoTime);
      kickTheBall = new KickBehavior(robotName,ros2Node,yoTime,yoDoubleSupport, fullHumanoidRobotModel,referenceFrames);
      this.environment = environment;
      this.yoDoubleSuport = yoDoubleSupport;
      yawAccordingToGoalPost = new WalkToLocationBehavior(robotName,ros2Node,fullHumanoidRobotModel,referenceFrames,wholeBodyControllerParameters.getWalkingControllerParameters());
      walkToLocationBehavior = new WalkToLocationBehavior(robotName,ros2Node, fullHumanoidRobotModel,referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      walkToOffsetPoint1 = new WalkToLocationBehavior(robotName,ros2Node,fullHumanoidRobotModel,referenceFrames,wholeBodyControllerParameters.getWalkingControllerParameters());
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
//      sphereDetctionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(robotName,"FiducialDetector",ros2Node,null);
      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      fiducialDetectorBehaviorService.setExpectedFiducialSize(0.2032);
      sphereLocationDetectionBehavior = new SearchForDoorBehavior(robotName,"DoorBehavior",ros2Node,null);
      addBehaviorService(fiducialDetectorBehaviorService);
      setupStateMachine();
   }



   @Override
   public void doControl()
   {
      super.doControl();
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Search and Kick Behavior");
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

         findBall = new BehaviorAction(sphereLocationDetectionBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               publishTextToSpeech("Entering door/sphere detection behavior");

            }

            @Override
            public void onExit()
            {
               publishTextToSpeech("Received Sphere/Door Location and entering the walking behavior");

               if(fiducialDetectorBehaviorService.getGoalHasBeenLocated())
               {
                  publishTextToSpeech("Detected fiducial");

               }
               super.onExit();
            }
         };
      }

      BehaviorAction walkTosphere1 = new BehaviorAction(walkToOffsetPoint1)
      {
         @Override protected void setBehaviorInput()
         {
            checkIfRobotIsAheadofBall();
            getDistanceFromSphereBeforeYawMotion();
            getoffsetPoint();
            publishTextToSpeech("Going to Waypoint 1");
            //need to write a method to calculate these waypoints.
            walkToOffsetPoint1.setTarget(getWaypoint());

            if(IS_ROBOT_AHEAD_OR_BACK)
            {
               walkToOffsetPoint1.setTarget(getWaypoint());
               IS_ROBOT_AHEAD_OR_BACK = false;
            }
         }
      };


         BehaviorAction walktowardstheObject = new BehaviorAction(walkToLocationBehavior)
         {

            @Override protected void setBehaviorInput ()
            {

               publishTextToSpeech("Directly Walking towards Sphere");
               FramePose2D pose1 = new FramePose2D(referenceFrames.getWorldFrame(),new Point2D(offsetFromSphere.getX()-0.15,offsetFromSphere.getY()),getWalkingYawofgetoffsetPoint());
               walkToLocationBehavior.setTarget(pose1);
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
            pelvis_up = true;
         }
      };



      //get pelvis to normal position
      BehaviorAction pelvisUpMotion = new BehaviorAction(atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         public void setBehaviorInput()
         {
//            publishTextToSpeech("Getting pelvis up by 10 cms");
            ReferenceFrame pelvisFrame = referenceFrames.getPelvisZUpFrame();
            FramePoint3D point = new FramePoint3D(pelvisFrame);
            point.changeFrame(referenceFrames.getWorldFrame());
            double nomialPelvisHeight = point.getZ();
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage();
            EuclideanTrajectoryPointMessage waypoint = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
            pelvisHeightTrajectoryMessage.setEnableUserPelvisControl(true);
            pelvisHeightTrajectoryMessage.setEnableUserPelvisControlDuringWalking(true);
            if(pelvis_up)
            {
               waypoint.getPosition().setZ(nomialPelvisHeight + 0.05);
               publishTextToSpeech("going up");
            }
            else
            {
               waypoint.getPosition().setZ(nomialPelvisHeight);
               publishTextToSpeech("going down");
            }
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            pelvis_up = false;
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

         //between the ball and the goal post
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.SETUP_ROBOT,resetrobot,WalkThroughDoorWOFiducialStates.SEARCH_FOR_SPHERE);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.SEARCH_FOR_SPHERE,findBall,WalkThroughDoorWOFiducialStates.WALK_TO_POINT_1);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_POINT_1,walkTosphere1, WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT); //added waypoint1
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT,walktowardstheObject, WalkThroughDoorWOFiducialStates.YAW_AS_PER_GOAL_POST);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.YAW_AS_PER_GOAL_POST, yawAction,WalkThroughDoorWOFiducialStates.PELVIS_UP);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.PELVIS_UP, pelvisUpMotion,WalkThroughDoorWOFiducialStates.KICK_ACTION);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.KICK_ACTION,kick,WalkThroughDoorWOFiducialStates.PELVIS_DOWN);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.PELVIS_DOWN,pelvisUpMotion,WalkThroughDoorWOFiducialStates.DONE);
         factory.addState(WalkThroughDoorWOFiducialStates.DONE,doneState);

      }

      else
      {
         //start adding them to the state factory
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.SETUP_ROBOT,resetrobot,WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.WALK_TO_THE_OBJECT, walktowardstheObject, WalkThroughDoorWOFiducialStates.YAW_AS_PER_GOAL_POST);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.YAW_AS_PER_GOAL_POST, yawAction,WalkThroughDoorWOFiducialStates.KICK_ACTION);
         factory.addStateAndDoneTransition(WalkThroughDoorWOFiducialStates.KICK_ACTION,kick,WalkThroughDoorWOFiducialStates.DONE);
         factory.addState(WalkThroughDoorWOFiducialStates.DONE,doneState);
      }

      //return the initial key

      return WalkThroughDoorWOFiducialStates.SETUP_ROBOT;
   }

   private boolean checkIfRobotIsAheadofBall()
   {
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(),0.0,0.0); //ensure that the 'y' of robot position in sim and AvatarStepUp hve to be same
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FramePoint2D initialSpherePoistion = new FramePoint2D(referenceFrames.getWorldFrame(),sphereLocationDetectionBehavior.getLocation().getPosition().getX(),
                                                            sphereLocationDetectionBehavior.getLocation().getPosition().getY());
      FrameVector2D whereToWalk = new FrameVector2D(referenceFrames.getWorldFrame());
      whereToWalk.set(initialSpherePoistion);
      whereToWalk.sub(robotPosition);
      double direction = Math.atan2(whereToWalk.getY(),whereToWalk.getX());
      if(robotPosition.getX() - initialSpherePoistion.getX() > 0)
      {
         this.IS_ROBOT_AHEAD_OR_BACK = true; //true means ahead of ball
         publishTextToSpeech("I am between the ball and the goal post");
      }
      else
      {
         this.IS_ROBOT_AHEAD_OR_BACK =false;
         publishTextToSpeech("I am behind the ball");
      }
      return IS_ROBOT_AHEAD_OR_BACK;
   }


   private double getDistanceFromSphereBeforeYawMotion()
   {
      FramePoint2D ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(),sphereLocationDetectionBehavior.getLocation().getPosition().getX(),
                                                     sphereLocationDetectionBehavior.getLocation().getPosition().getY());
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(), 0.0, 0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(referenceFrames.getWorldFrame());
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      //calculating d value
      distanceFromSphereBeforeYawMotion = Math.sqrt(Math.pow(walkingDirection.getX(),2.0) + Math.pow(walkingDirection.getY(),2.0));
      return distanceFromSphereBeforeYawMotion;
   }

   private FramePose2D getWaypoint()
   {
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(),0.0,0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FramePoint2D spherePos = new FramePoint2D(ReferenceFrame.getWorldFrame(),sphereLocationDetectionBehavior.getLocation().getPosition().getX(),
                                                sphereLocationDetectionBehavior.getLocation().getPosition().getY());
      FrameVector2D vector1 = new FrameVector2D(referenceFrames.getWorldFrame());
      vector1.set(spherePos);
      vector1.sub(robotPosition);
      vector1.normalize();
      double walkingYaw = Math.atan2(vector1.getY(),vector1.getX());
      double x11 = spherePos.getX() - 1.5; // -3.0 can be set as a safety factor
      double y11;
      if((robotPosition.getY() - spherePos.getY()) < 0)
      {
         y11 = spherePos.getY() - 1.0;
      }
      else
      {
         y11 = spherePos.getY() + 1.0;
      }
      FramePose2D ret = new FramePose2D(referenceFrames.getWorldFrame(),new Point2D(x11,y11),walkingYaw);
      if(!IS_ROBOT_AHEAD_OR_BACK)
      {
         ret = new FramePose2D(referenceFrames.getWorldFrame(),new Point2D(robotPosition),0.0);
      }
      return ret;
   }



   private FramePose2D getPositionwrtGoalPost()
   {
//      FramePoint2D goalPostPosition = new FramePoint2D(ReferenceFrame.getWorldFrame(),environment.getGoalPostPosition().getX(),environment.getGoalPostPosition().getY());
//      if(fiducialDetectorBehaviorService.getGoalHasBeenLocated())
//      {
//         publishTextToSpeech("Detected fiducial for Yaw motion ");
//      }

      fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(tmpGFP);
      goalPostPose = new Pose3D(tmpGFP);
      FramePoint2D goalPostPosition = new FramePoint2D(ReferenceFrame.getWorldFrame(),goalPostPose.getPosition().getX(),goalPostPose.getPosition().getY());
      FramePoint2D robotPosition = new FramePoint2D(referenceFrames.getMidFeetZUpFrame(),0.0,0.0);
      robotPosition.changeFrame(referenceFrames.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(referenceFrames.getWorldFrame());
      walkingDirection.set(goalPostPosition);
      walkingDirection.sub(robotPosition);
      FrameVector2D tmp = getWalkingDirectionForYawMotion();
      //calcutating d value
      double distanceFromSphereBeforeYawMotion1 = Math.sqrt(Math.pow(tmp.getX(),2.0) + Math.pow(tmp.getY(),2.0)); //the issue is most probably with the tmp value
//      walkingDirection.set(robotPosition);
//      walkingDirection.sub(goalPostPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
//      if(DEBUG)
//      {
         System.out.println("goal Post Position :" + goalPostPosition.getX() + goalPostPosition.getY());
//      }

      double xprime;
      double yprime;
//      xprime = offsetFromSphere.getX()+ (offsetFromSphere.getX() - (offsetFromSphere.getX()*Math.cos(walkingYaw))) + 0.1;
//      yprime = offsetFromSphere.getY() - offsetFromSphere.getY()*Math.sin(walkingYaw);
      xprime = offsetFromSphere.getX() + (distanceFromSphereBeforeYawMotion1 - distanceFromSphereBeforeYawMotion1*Math.cos(walkingYaw));
      yprime = offsetFromSphere.getY() - (distanceFromSphereBeforeYawMotion1*Math.sin(walkingYaw));

      offsetFromSphereForGoalPost = new Point2D(xprime,yprime);
      FramePose2D poseTowalkTowrtGoalPost = new FramePose2D(referenceFrames.getWorldFrame(),offsetFromSphereForGoalPost,walkingYaw);

      return poseTowalkTowrtGoalPost;
   }

   private FramePose2D getoffsetPoint() //this is my d variable
   {
      //use walk to interactactable behavior which by deffult assume two waypoints
      FramePoint2D ballPosition2d;
      if(!BALL_DETECTION)
      {
         ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), environment.getInitialSpherePosition(),
                                                                    0.0);
      }
      else
      {
         ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(),sphereLocationDetectionBehavior.getLocation().getPosition().getX(),
                                                        sphereLocationDetectionBehavior.getLocation().getPosition().getY());
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
      this.walkingYawofgetoffsetPoint = walkingYaw;
      if(DEBUG)
      {
         System.out.println(walkingYaw);
      }

      double x;
      double y;
      double SphereRadius = 0.25;
      if(SphereRadius >= 0.5)
      {
         x = ballPosition2d.getX() - (SphereRadius+ 0.25);
         y = ballPosition2d.getY() - (0.25*Math.tan(walkingYaw));
      }
      else if (SphereRadius >= 0.25)
      {
         x = ballPosition2d.getX() - (SphereRadius + 0.15);
         y = ballPosition2d.getY() - (0.15*Math.tan(walkingYaw));
      }

      else
      {
         x = ballPosition2d.getX() - (SphereRadius + 0.1);
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

   public double getWalkingYawofgetoffsetPoint()
   {
      return walkingYawofgetoffsetPoint;
   }
}
