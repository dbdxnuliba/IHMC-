package us.ihmc.avatar.pushRecovery;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.Test;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.util.Random;

import static org.junit.Assert.assertTrue;

public abstract class AvatarICPOptimizationPushRecoveryATest extends AvatarICPOptimizationPushRecoveryTestSetup
{
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOneLegStanding() throws Exception
   {
      FootstepDataListMessage footsteps = createTwoStepStandingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1*swingTime;
      //pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      pushRobotController.applyForce(forceDirection,magnitude,duration);
      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationDiagonalPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, -0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05*swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationDiagonalPushFrontalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05*swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationLongInwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(true);
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.1 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.8 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushInTransfer() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();

      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * transferTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.7);
      assertTrue(success);

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationInwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footstepDataListMessage = createForwardWalkingFootstepMessage();
      setupAndRunTest(footstepDataListMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footstepDataListMessage);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationForwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationForwardPushInSlowSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createSlowForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationBackwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushOnEachStep() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:

      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D firstForceDirection = new Vector3D(0.0, -1.0, 0.0);
      Vector3D secondForceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);
      boolean success;

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      assertTrue(success);

      validateTest(footsteps, false);
   }
   protected double getLiftOffHeight()
   {
      return 0.15;
   }

   //the center of the trajectory
   protected Point3D getCircleCenterFromAnkle(RobotSide robotSide)
   {
      return new Point3D(0.0, 0.0, 0.15);
   }

   //how far out does the trajectory span from the circle center
   protected Vector3D getCircleRadius()
   {
      return new Vector3D(0.15, 0.15, 0.08);
   }

   //get a random position, if the radius and center are well tunes, this should be kinematically feasible
   private Point3D getRandomPositionInSphere(Random random, RobotSide robotSide)
   {
      Point3D circleCenterFromAnkle = getCircleCenterFromAnkle(robotSide);
      Vector3D circleRadius = getCircleRadius();

      Point3D min = new Point3D();
      Point3D max = new Point3D();

      min.sub(circleCenterFromAnkle, circleRadius);
      max.add(circleCenterFromAnkle, circleRadius);

      return RandomGeometry.nextPoint3D(random, min, max);
   }

   //The first step in the test, send a FootTrajectoryMessage with a single point and simulate
   private boolean pickupFoot(RobotSide robotSide, RigidBody foot) throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double timeToPickupFoot = 1.0;

      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, getLiftOffHeight());
      footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools
            .createFootTrajectoryMessage(robotSide, timeToPickupFoot, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToPickupFoot + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
   }

   //Put the foot back on the ground, this doesn't have any special ground checks, it's just easier to read this way
   private boolean putFootOnGround(RobotSide robotSide, RigidBody foot, FramePose3D desiredPose) throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double trajectoryTime = 1.0;

      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPose.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      FootLoadBearingMessage loadBearingMessage = new FootLoadBearingMessage();
      loadBearingMessage.setRobotSide(robotSide.toByte());
      loadBearingMessage.setLoadBearingRequest(LoadBearingRequest.LOAD.toByte());
      drcSimulationTestHelper.publishToController(loadBearingMessage);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.2 + trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
   }

   //moves the foot to a position using getRandomPositionInSphere
   private void moveFootToRandomPosition(Random random, RobotSide robotSide, RigidBody foot, SimulationConstructionSet scs)
         throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double trajectoryTime = 1.0;
      String bodyName = foot.getName();

      FramePose3D desiredRandomFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomFootPose.setPosition(getRandomPositionInSphere(random, robotSide));
      desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomFootPose.get(desiredPosition, desiredOrientation);
      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      boolean result = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime()+5);
      //assertTrue(result);

      //EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(bodyName, desiredPosition, desiredOrientation, scs);
   }
}
