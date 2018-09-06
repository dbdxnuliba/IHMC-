package us.ihmc.valkyrie.straightLegWalking;

import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.*;

public class ValkyrieStraightLegWalkingTest extends AvatarStraightLegWalkingTest
{
   private final ValkyrieRobotModel valkyrieRobotModel = new MyValkyrieRobotModel();

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
   @Test(timeout = 200000000)
   public void testWalkingOverCinderBlockField() throws Exception
   {
      super.testWalkingOverCinderBlockField();
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
   @ContinuousIntegrationTest(estimatedDuration = 167.7)
   @Test(timeout = 680000)
   public void testSteppingDown() throws SimulationExceededMaximumTimeException
   {
      super.testSteppingDown();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 200000)
   public void testSteppingDownEveryTime() throws Exception
   {
      super.testSteppingDownEveryTime();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 200000)
   public void testRandomHeightField() throws Exception
   {
      super.testRandomHeightField();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return valkyrieRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   private class MyValkyrieRobotModel extends ValkyrieRobotModel
   {
      public MyValkyrieRobotModel()
      {
         super(RobotTarget.SCS,false);
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new TestWalkingControllerParameters(getJointMap());
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new TestICPPlannerParameters();
      }
   }

   private class TestWalkingControllerParameters extends ValkyrieWalkingControllerParameters
   {
      private final ValkyrieJointMap jointMap;


      public TestWalkingControllerParameters(ValkyrieJointMap jointMap)
      {
         super(jointMap, RobotTarget.SCS);

         this.jointMap = jointMap;

      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportX()
      {
         return 0.04;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportY()
      {
         return 0.02;
      }

      @Override
      public boolean controlHeightWithMomentum()
      {
         return false;
      }

      @Override
      public boolean applySecondaryJointScaleDuringSwing()
      {
         return true;
      }

      @Override
      public LeapOfFaithParameters getLeapOfFaithParameters()
      {
         return new TestLeapOfFaithParameters();
      }

      @Override
      public LegConfigurationParameters getLegConfigurationParameters()
      {
         return new TestLegConfigurationParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new TestMomentumOptimizationSettings(jointMap);
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new TestSwingTrajectoryParameters();
      }

      @Override
      public TestToeOffParameters getToeOffParameters()
      {
         return new TestToeOffParameters(jointMap);
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return new TestSteppingParameters(jointMap);
      }

   }

   private class TestToeOffParameters extends ValkyrieToeOffParameters
   {
      public TestToeOffParameters(ValkyrieJointMap jointMap)
      {
         super(false);
      }

      @Override
      public boolean checkCoPLocationToTriggerToeOff()
      {
         return false;
      }

      @Override
      public double getCoPProximityForToeOff()
      {
         return 0.05;
      }

      @Override
      public double getICPPercentOfStanceForDSToeOff()
      {
         return 0.20;
      }

      @Override
      public double getICPPercentOfStanceForSSToeOff()
      {
         return 0.10;
      }

      @Override
      public boolean checkECMPLocationToTriggerToeOff()
      {
         return false;
      }

      @Override
      public double getECMPProximityForToeOff()
      {
         return 0.01;
      }

      @Override
      public boolean doToeOffIfPossibleInSingleSupport()
      {
         return true;
      }

      @Override
      public double getAnkleLowerLimitToTriggerToeOff()
      {
         return -0.75;
      }
   }

   private class TestSwingTrajectoryParameters extends ValkyrieSwingTrajectoryParameters
   {
      public TestSwingTrajectoryParameters()
      {
         super(RobotTarget.SCS);
      }

      @Override
      public boolean useSingularityAvoidanceInSwing()
      {
         return false;
      }

      @Override
      public boolean useSingularityAvoidanceInSupport()
      {
         return false;
      }

      @Override
      public boolean doHeelTouchdownIfPossible()
      {
         return true;
      }

      @Override
      public boolean doToeTouchdownIfPossible()
      {
         return true;
      }

      @Override
      public boolean addOrientationMidpointForObstacleClearance()
      {
         return true;
      }
   }

   private class TestLeapOfFaithParameters extends LeapOfFaithParameters
   {
      @Override
      public boolean scaleFootWeight()
      {
         return true;
      }

      @Override
      public boolean usePelvisRotation()
      {
         return true;
      }

      @Override
      public double getMinimumPelvisWeight()
      {
         return 0.5;
      }
   }

   private class TestLegConfigurationParameters extends ValkyrieLegConfigurationParameters
   {
      public TestLegConfigurationParameters()
      {
         super(false);
      }

      @Override
      public boolean attemptToStraightenLegs()
      {
         return true;
      }

      @Override
      public LegConfigurationGains getBentLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setJointSpaceKp(150.0);
         gains.setJointSpaceKd(6.0);

         return gains;
      }

      @Override
      public LegConfigurationGains getStraightLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setJointSpaceKp(750.0);
         gains.setJointSpaceKd(10.0);

         return gains;
      }
   }

   private class TestMomentumOptimizationSettings extends ValkyrieMomentumOptimizationSettings
   {
      public TestMomentumOptimizationSettings(ValkyrieJointMap jointMap)
      {
         super(jointMap);
      }

      @Override
      public double getJointAccelerationWeight()
      {
         return 0.05;
      }
   }

   private class TestICPPlannerParameters extends ValkyrieCapturePointPlannerParameters
   {
      public TestICPPlannerParameters()
      {
         super(false);
      }

      @Override
      public double getExitCoPForwardSafetyMarginOnToes()
      {
         return 0.015;
      }

      @Override
      public boolean putExitCoPOnToes()
      {
         return true;
      }
   }

   private class TestSteppingParameters extends ValkyrieSteppingParameters
   {
      public TestSteppingParameters(ValkyrieJointMap jointMap)
      {
         super(RobotTarget.SCS);
      }

      @Override
      public double getMaxStepLength()
      {
         return 1.0;
      }
   }

   public static void main(String[] args) throws Exception
   {

   }
}
