package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCPushRecoveryStandingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Disabled
public class ValkyriePushRecoveryStandingTest extends DRCPushRecoveryStandingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Disabled("Needs to be improved")
   @Test// timeout = 270000
   @Override
   public void testDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testDoublePushForwardInDoubleSupportAndContinueWalking();
   }

   @Test// timeout = 270000
   @Override
   public void testPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushBackwardForwardInDoubleSupportAndContinueWalking();
   }

   @Test// timeout = 270000
   @Override
   public void testPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushBackwardInDoubleSupportAndContinueWalking();
   }

   @Test// timeout = 270000
   @Override
   public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushForwardInDoubleSupport();
   }

   @Test// timeout = 270000
   @Override
   public void testPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushForwardInDoubleSupportAndContinueWalking();
   }

}
