package us.ihmc.avatar.straightLegWalking;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryTestSetup;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public abstract class AvatarPrivilegedConfigurationPushRecoveryTest extends AvatarICPOptimizationPushRecoveryTestSetup implements MultiRobotTestInterface
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
