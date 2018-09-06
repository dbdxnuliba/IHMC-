package us.ihmc.avatar.straightLegWalking;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryTestSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarLegLengthControlTest extends AvatarICPOptimizationPushRecoveryTestSetup implements MultiRobotTestInterface
{

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushStanding() throws Exception
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
   public void testPushDiagonalInSwing() throws Exception
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
      setupAndRunTest(footsteps);
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
   public void testLongStepsPushWithLegLengthControl() throws Exception
   {
      FootstepDataListMessage footsteps = createLongStepsForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.05 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }
}
