package us.ihmc.atlas.straightLegWalking;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.atlas.straightLegWalking.AtlasStraightLegWalkingTest.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarLegLengthControlTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLegLengthControlTest extends AvatarLegLengthControlTest
{

   private final AtlasRobotModel atlasRobotModel = new MyAtlasRobotModel();

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushStanding() throws Exception
   {
      percentWeight = 0.5;
      super.testPushStanding();
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
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushDiagonalFrontalInSwing() throws Exception
   {
      percentWeight = 1.3;
      super.testPushDiagonalFrontalInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testLongStepsPushWithLegLengthControl() throws Exception
   {
      percentWeight = 0.0;
      super.testLongStepsPushWithLegLengthControl();



   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   protected double getSizeScale()
   {
      return 1;
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
         return new AtlasLegLengthControlTest.TestWalkingControllerParameters(getJointMap(), getContactPointParameters());
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new AtlasLegLengthControlTest.TestICPPlannerParameters(getPhysicalProperties());
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
      public double getOmega0()
      {
         return 3.0/Math.sqrt(1);
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
         return new AtlasLegLengthControlTest.TestLeapOfFaithParameters();
      }

      @Override
      public LegConfigurationParameters getLegConfigurationParameters()
      {
         return new AtlasLegLengthControlTest.TestLegConfigurationParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new AtlasLegLengthControlTest.TestMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new AtlasLegLengthControlTest.TestSwingTrajectoryParameters();
      }

      @Override
      public AtlasLegLengthControlTest.TestToeOffParameters getToeOffParameters()
      {
         return new AtlasLegLengthControlTest.TestToeOffParameters(jointMap);
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return new AtlasLegLengthControlTest.TestSteppingParameters(jointMap);
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
         gains.setJointSpaceKp(100.0);
         gains.setJointSpaceKd(6.0);

         return gains;
      }

      /** {@inheritDoc} */
      @Override
      public LegConfigurationGains getStraightLegGains()
      {
         /*
         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setActuatorSpaceKp(1000.0);
         //gains.setActuatorSpaceKd(400);
        // gains.setJointSpaceKp(400);
         gains.setJointSpaceKd( 30.0);
         */

         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setJointSpaceKp(100.0);
         gains.setJointSpaceKd(6.0);

         return gains;
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

   public static void main(String[] args) throws Exception
   {
      AtlasStraightLegWalkingTest test = new AtlasStraightLegWalkingTest();
      test.testSteppingDown();
   }
}
