package us.ihmc.valkyrie.straightLegWalking;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.*;

public class ValkyrieStraightLegWalkingTest extends AvatarStraightLegWalkingTest
{
   private final ValkyrieRobotModel valkyrieRobotModel = new ValkyrieStraightLegRobotModel(RobotTarget.SCS,false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testForwardWalking() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalking();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 400000)
   public void testSlowerWalking() throws SimulationExceededMaximumTimeException
   {
      super.testSlowerWalking();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 167.7)
   @Test(timeout = 520000)
   public void testWalkingOverStairs() throws Exception
   {
      super.testWalkingOverStairs();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 200000)
   public void testDropOffsWhileWalking() throws SimulationExceededMaximumTimeException
   {
      super.testDropOffsWhileWalking();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return valkyrieRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Valkyrie";
   }
}
