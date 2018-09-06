package us.ihmc.valkyrie.privilegedConfigurationWalking;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryATest;
import us.ihmc.avatar.straightLegWalking.AvatarPrivilegedConfigurationPushRecoveryTest;
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
import us.ihmc.valkyrie.privilegedConfigurationWalking.ValkyrieStraightLegWalkingTest.*;

public class ValkyriePriviligedConfigurationPushRecoveryTest extends AvatarPrivilegedConfigurationPushRecoveryTest
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
   public double getNominalHeight()
   {
      return 0;
   }

   @Override
   public double getSlowTransferDuration()
   {
      return 0;
   }

   @Override
   public double getSlowSwingDuration()
   {
      return 0;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return valkyrieRobotModel;
   }

   @Override
   protected double getSizeScale()
   {
      return 0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Valkyrie";
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
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestWalkingControllerParameters(getJointMap());
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestICPPlannerParameters();
      }
   }

   private class TestWalkingControllerParameters extends ValkyrieWalkingControllerParameters
   {
      private final ValkyrieJointMap jointMap;


      public TestWalkingControllerParameters(ValkyrieJointMap jointMap)
      {
         super(jointMap,RobotTarget.SCS);

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
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestLeapOfFaithParameters();
      }

      @Override
      public LegConfigurationParameters getLegConfigurationParameters()
      {
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestLegConfigurationParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestMomentumOptimizationSettings(jointMap);
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestSwingTrajectoryParameters();
      }

      @Override
      public ValkyriePriviligedConfigurationPushRecoveryTest.TestToeOffParameters getToeOffParameters()
      {
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestToeOffParameters(jointMap);
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return new ValkyriePriviligedConfigurationPushRecoveryTest.TestSteppingParameters(jointMap);
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
         gains.setJointSpaceKp(40.0);
         gains.setJointSpaceKd(6.0);

         return gains;
      }

      @Override
      public double getAccelerationForSupportKneeStraightening()
      {
         return 10.0;
      }

      @Override
      public double getSupportKneeCollapsingDurationFractionOfStep()
      {
         return 0.5;
         //      return 1.0;
         //      return 0.1; // for big step down
      }

      @Override
      public double getKneeAngleWhenStraight()
      {
         return 0.25;
      }

      @Override
      public double getKneeAngleWhenBracing()
      {
         return 0.4;
      }

      @Override
      public double getKneeAngleWhenExtended()
      {
         return 0.0;
      }

      @Override
      public double getDesiredFractionOfMidrangeForCollapsedAngle()
      {
         return 0.3;
         //      return 1.8; // for big step down
      }

      @Override
      public double getFractionOfSwingToStraightenLeg()
      {
         return 0.4;
      }

      @Override
      public double getFractionOfTransferToCollapseLeg()
      {
         return 0.7;
         //      return 1.0;
      }

      @Override
      public double getFractionOfSwingToCollapseStanceLeg()
      {
         return 0.55;
         //      return 1.0;
         //      return 0.3; //for big step down
      }

      @Override
      public double getLegPrivilegedLowWeight()
      {
         return 5.0;
      }

      @Override
      public double getLegPrivilegedMediumWeight()
      {
         return 50.0;
      }

      @Override
      public double getLegPrivilegedHighWeight()
      {
         return 150.0;
      }

      @Override
      public double getPrivilegedMaxVelocity()
      {
         return Double.POSITIVE_INFINITY;
      }

      @Override
      public double getPrivilegedMaxAcceleration()
      {
         return Double.POSITIVE_INFINITY;
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
}
