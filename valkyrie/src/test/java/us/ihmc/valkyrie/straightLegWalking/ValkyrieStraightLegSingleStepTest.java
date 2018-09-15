package us.ihmc.valkyrie.straightLegWalking;

import org.junit.Test;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegSingleStepTest;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegSingleStepTest;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.*;
import us.ihmc.valkyrie.straightLegWalking.ValkyrieStraightLegWalkingTest.*;

import java.util.EnumMap;

public class ValkyrieStraightLegSingleStepTest extends AvatarStraightLegSingleStepTest
{
      private final ValkyrieRobotModel valkyrieRobotModel = new ValkyrieStraightLegRobotModel(RobotTarget.SCS, false);

      @ContinuousIntegrationTest(estimatedDuration = 45.0)
      @Test(timeout = 70000)
      public void testForwardStep() throws SimulationExceededMaximumTimeException
      {
         double stepLength = 1.3;
         double stepWidth = 0.25;

         setStepLength(stepLength);
         setStepWidth(stepWidth);

         super.testForwardStep();
      }

      @ContinuousIntegrationTest(estimatedDuration = 45.0)
      @Test(timeout = 70000)
      public void testForwardStepWithPause() throws SimulationExceededMaximumTimeException
      {
         double stepLength = 1.0;
         double stepWidth = 0.25;

         setStepLength(stepLength);
         setStepWidth(stepWidth);

         super.testForwardStepWithPause();
      }

      @ContinuousIntegrationTest(estimatedDuration = 45.0)
      @Test(timeout = 70000)
      public void testWideStep() throws SimulationExceededMaximumTimeException
      {
         double stepWidth = 0.6;
         double stanceWidth = 0.25;

         setStepWidth(stepWidth);
         setStanceWidth(stanceWidth);

         super.testWideStep();
      }

      @ContinuousIntegrationTest(estimatedDuration = 50.0)
      @Test(timeout = 100000)
      public void testSteppingDown() throws SimulationExceededMaximumTimeException
      {
         double stepHeight = 0.4;
         double stepLength = 0.4;
         double stanceWidth = 0.25;
         setStepDownHeight(stepHeight);
         setStepHeight(stepHeight);
         setStepLength(stepLength);
         setStanceWidth(stanceWidth);
         super.testSteppingDown();
      }

      @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
      @Test(timeout = 100000)
      public void testSteppingDownWithClosing() throws SimulationExceededMaximumTimeException
      {
         double stepDownHeight = 0.3;
         double stepLength = 0.4;
         double stanceWidth = 0.25;
         setStepDownHeight(stepDownHeight);
         setStepLength(stepLength);
         setStanceWidth(stanceWidth);
         super.testSteppingDownWithClosing();
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
