package us.ihmc.avatar.straightLegWalking;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryTestSetup;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

public abstract class AvatarPrivilegedConfigurationPushRecoveryTest extends AvatarICPOptimizationPushRecoveryTestSetup implements MultiRobotTestInterface
{
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushFrontalStanding() throws Exception
   {
      FootstepDataListMessage footsteps = createStandingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);


      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);


      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05;
      pushRobotController.applyForce(forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushDiagonalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);

      drcSimulationTestHelper.getSimulationConstructionSet().setupVarGroup("ICPTestVars", new String[]{"desiredICPX", "desiredICPY","perfectCMPX","perfectCMPY","centerOfMassX","centerOfMassY"});
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, -0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushDiagonalFrontalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushFrontalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushBackInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushRightInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushLeftInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupAndRunTest(footsteps, flatGround);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }
}
