package us.ihmc.atlas.straightLegWalking;

import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.atlas.straightLegWalking.AtlasStraightLegWalkingTest.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarPrivilegedConfigurationPushRecoveryTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AtlasPriviligedConfigurationPushRecoveryTest extends AvatarPrivilegedConfigurationPushRecoveryTest
{
   private final RobotTarget target = RobotTarget.SCS;
   private final AtlasRobotModel atlasRobotModel = new MyAtlasRobotModel();
   private final AtlasRobotModel atlasRobotModelNormal = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false);
   private final boolean useNormal = true;
   private final double privilegedAngleWhenStraight = 1.177;

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushFrontalStanding() throws Exception
   {
      percentWeight = 0.62;
      super.testPushFrontalStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushDiagonalInSwing() throws Exception
   {
      percentWeight = 0.5;
      super.testPushDiagonalInSwing();
   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @Override
   public double getSlowTransferDuration()
   {
      return 0.15;
   }

   @Override
   public double getSlowSwingDuration()
   {
      return 0.6;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      if(useNormal)
         return atlasRobotModelNormal;
      else
      return atlasRobotModel;
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   private class MyAtlasRobotModel extends AtlasRobotModel
   {
      public MyAtlasRobotModel()
      {
         super(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestWalkingControllerParameters(getJointMap(), getContactPointParameters());
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestICPPlannerParameters(getPhysicalProperties());
      }
   }

   private class TestWalkingControllerParameters extends AtlasWalkingControllerParameters
   {
      private final AtlasJointMap jointMap;
      private final AtlasContactPointParameters contactPointParameters;

      public TestWalkingControllerParameters(AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(RobotTarget.SCS, jointMap, contactPointParameters);

         this.jointMap = jointMap;
         this.contactPointParameters = contactPointParameters;
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
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestLeapOfFaithParameters();
      }

      @Override
      public LegConfigurationParameters getLegConfigurationParameters()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestLegConfigurationParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestSwingTrajectoryParameters();
      }

      @Override
      public AtlasPriviligedConfigurationPushRecoveryTest.TestToeOffParameters getToeOffParameters()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestToeOffParameters(jointMap);
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return new AtlasPriviligedConfigurationPushRecoveryTest.TestSteppingParameters(jointMap);
      }

   }

   private class TestToeOffParameters extends AtlasToeOffParameters
   {
      public TestToeOffParameters(AtlasJointMap jointMap)
      {
         super(jointMap);
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

   private class TestSwingTrajectoryParameters extends AtlasSwingTrajectoryParameters
   {
      public TestSwingTrajectoryParameters()
      {
         super(RobotTarget.SCS, 1.0);
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

   private class TestLegConfigurationParameters extends AtlasLegConfigurationParameters
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
         //gains.setJointSpaceKp(1000.0);
         gains.setActuatorSpaceKp(1000);
         gains.setJointSpaceKd(20.0);

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
         return privilegedAngleWhenStraight;
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

   }

   private class TestMomentumOptimizationSettings extends AtlasMomentumOptimizationSettings
   {
      public TestMomentumOptimizationSettings(AtlasJointMap jointMap, int numberOfContactableBodies)
      {
         super(jointMap, numberOfContactableBodies);
      }

      @Override
      public double getJointAccelerationWeight()
      {
         return 0.05;
      }
   }

   private class TestICPPlannerParameters extends AtlasSmoothCMPPlannerParameters
   {
      public TestICPPlannerParameters(AtlasPhysicalProperties physicalProperties)
      {
         super(physicalProperties);
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

   private class TestSteppingParameters extends AtlasSteppingParameters
   {
      public TestSteppingParameters(AtlasJointMap jointMap)
      {
         super(jointMap);
      }

      @Override
      public double getMaxStepLength()
      {
         return 1.0;
      }
   }
}
