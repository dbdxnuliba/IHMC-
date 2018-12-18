package us.ihmc.valkyrie.heightForBalance;

import org.junit.Test;
import org.opencv.core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.heightForBalanceTest.AvatarHeightForBalanceTest;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.*;

public class ValkyrieHeightForBalanceTest extends AvatarHeightForBalanceTest
{
   // Which robot model to use. Default uses the VaryingHeight/HeightForBalance controller
   private final boolean useNormalRobot = false;
   private final boolean useQPbasedModel = false;
   private ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS,false);
   private HeightForBalanceModel heightForBalanceModel = new HeightForBalanceModel();
   private QPForHeightValkyrieRobotModel qpForHeightModel = new QPForHeightValkyrieRobotModel();

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushStanding() throws Exception
   {
      // No-height max recoverable percentWeight: 0.88
      percentWeight =0.201;//0.201;//0.184;
      super.testPushStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 10000000)
   public void testPushAngle() throws Exception
   {
      angle = 3.14159265358979;
      percentWeight = 1.28;
      super.testPushAngle();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushFrontalInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 1.36
      percentWeight = 1.45;
      super.testPushFrontalInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushBackInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.85
      percentWeight = 0.70;
      super.testPushBackInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushRightInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.62
      percentWeight = 0.62;
      super.testPushRightInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushLeftInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.54
      percentWeight = 0.57;
      super.testPushLeftInSwing();
   }


   @Override
   @Test
   public void testIterativePushStanding() throws Exception
   {
      percentWeight= 0.579;
      super.testIterativePushStanding();
   }

   @Override
   public double getNominalHeight()
   {
      return 0.8;
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
      if(useNormalRobot)
         return valkyrieRobotModel;
      else if(useQPbasedModel)
         return  qpForHeightModel;
      else
         return heightForBalanceModel;
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Valkyrie";
   }

   private class HeightForBalanceModel extends ValkyrieRobotModel
   {
      public HeightForBalanceModel(){
         super(RobotTarget.SCS,false);
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new heightForBalanceWalkingControllerParameters(getJointMap());
      }
   }
   private class heightForBalanceWalkingControllerParameters extends ValkyrieWalkingControllerParameters
   {
      private final ValkyrieJointMap jointMap;

      public heightForBalanceWalkingControllerParameters(ValkyrieJointMap jointMap)
      {
         super(jointMap);
         this.jointMap = jointMap;
      }
      @Override
      public boolean useHeightForBalanceController()
      {
         return true;
      }
   }



   private class QPForHeightValkyrieRobotModel extends ValkyrieRobotModel
   {
      public QPForHeightValkyrieRobotModel()
      {
         super(RobotTarget.SCS,false);
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new TestWalkingControllerParameters(getJointMap());
      }
   }

   private class TestWalkingControllerParameters extends ValkyrieWalkingControllerParameters
   {
      private final ValkyrieJointMap jointMap;

      public TestWalkingControllerParameters(ValkyrieJointMap jointMap)
      {
         super(jointMap);

         this.jointMap = jointMap;
      }

      @Override
      public boolean controlHeightWithMomentum()
      {
         return true;
      }

      @Override
      public boolean applySecondaryJointScaleDuringSwing()
      {
         return true;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         return  false;
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
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return new TestICPOptimizationParameters();
      }

   }

   private class TestLegConfigurationParameters extends ValkyrieLegConfigurationParameters
   {
      public TestLegConfigurationParameters()
      {
         super(RobotTarget.SCS);
      }

      @Override
      public boolean attemptToStraightenLegs()
      {
         return false;
      }

      @Override
      public LegConfigurationGains getBentLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setJointSpaceKp(500.0);
         gains.setJointSpaceKd(15.0);

         return gains;
      }


      @Override
      public LegConfigurationGains getStraightLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         //gains.setJointSpaceKp(1000.0);
         gains.setActuatorSpaceKp(1000);
         gains.setJointSpaceKd(35.0);

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
         return 0.15;
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

   }

   private class TestICPOptimizationParameters extends ValkyrieICPOptimizationParameters
   {
      public TestICPOptimizationParameters()
      {
         super(RobotTarget.SCS);
      }

      @Override
      public boolean allowStepAdjustment()
      {
         return false;
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

      @Override
      public Vector3D getLinearMomentumWeight()
      {
         Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.001);
         return linearMomentumWeight;
      }
   }
}
