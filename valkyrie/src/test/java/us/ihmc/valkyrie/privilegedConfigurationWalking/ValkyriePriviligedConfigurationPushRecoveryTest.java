package us.ihmc.valkyrie.privilegedConfigurationWalking;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryATest;
import us.ihmc.avatar.straightLegWalking.AvatarPrivilegedConfigurationPushRecoveryTest;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.*;
import us.ihmc.valkyrie.privilegedConfigurationWalking.ValkyrieStraightLegWalkingTest.*;

import java.util.ArrayList;
import java.util.List;

public class ValkyriePriviligedConfigurationPushRecoveryTest extends AvatarPrivilegedConfigurationPushRecoveryTest
{
   private final RobotTarget target = RobotTarget.SCS;
   private final ValkyrieRobotModel valkyrieRobotModel = new MyValkyrieRobotModel();
   private final ValkyrieRobotModel valkyrieRobotModelNormal = new ValkyrieRobotModel(target,false);
   private final boolean useNormalRobotModel = true;
   private final double privilegedAngleWhenStraight = 1.17;


   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushFrontalStanding() throws Exception
   {
      percentWeight = 0.57;
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
      return 0.7;
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
      if(useNormalRobotModel)
      return valkyrieRobotModelNormal;
      else
         return valkyrieRobotModel;
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

   private class MyValkyrieRobotModel extends ValkyrieRobotModel
   {
      public MyValkyrieRobotModel()
      {
         super(target,false);
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
         super(jointMap,target);

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


      @Override
      public ICPControlGains createICPControlGains()
      {
         ICPControlGains gains = new ICPControlGains();
         boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

         double kpOrthogonal = runningOnRealRobot ? 1.9 : 1.5;
         double kpParallel = runningOnRealRobot ? 2.0 : 2.5;
         double ki = runningOnRealRobot ? 0.0 : 0.0;
         double kiBleedOff = 0.9;

         gains.setKpParallelToMotion(kpParallel);
         gains.setKpOrthogonalToMotion(kpOrthogonal);
         gains.setKi(ki);
         gains.setIntegralLeakRatio(kiBleedOff);

         if (target == RobotTarget.REAL_ROBOT)
            gains.setFeedbackPartMaxRate(1.5);

         return gains;
      }

      @Override
      public PDGains getCoMHeightControlGains()
      {
         PDGains gains = new PDGains();

         boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

         double kp = runningOnRealRobot ? 40.0 : 50.0;
         double zeta = runningOnRealRobot ? 0.4 : 1.0;
         double maxAcceleration = 0.5 * 9.81;
         double maxJerk = maxAcceleration / 0.05;

         gains.setKp(kp);
         gains.setZeta(zeta);
         gains.setMaximumFeedback(maxAcceleration);
         gains.setMaximumFeedbackRate(maxJerk);

         return gains;
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
         super(target);
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
         gains.setActuatorSpaceKp(200.0);
         gains.setActuatorSpaceKd(20.0);

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
         return 100.0;
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
         super(target);
      }

      @Override
      public double getMaxStepLength()
      {
         return 1.0;
      }
   }

}
