package us.ihmc.valkyrie.heightForBalance;

import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.heightForBalanceTest.AvatarHeightForBalanceIterativeTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieMomentumOptimizationSettings;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieHeightForBalanceIterativeTest extends AvatarHeightForBalanceIterativeTest
{

   @Override
   @Test
   public void testIterativePush() throws Exception
   {
      super.testIterativePush();
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
   protected DRCRobotModel getRobotModel()
   {
      if(useNormalRobot)
         return new NormalModel();
      else
         return new HeightForBalanceModel();
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   private class NormalModel extends ValkyrieRobotModel
   {
      public NormalModel(){super(RobotTarget.SCS,false);}

      @Override
      public WalkingControllerParameters getWalkingControllerParameters(){return new NormalModelWalkingControllerParameters(getJointMap());}
   }
   private class NormalModelWalkingControllerParameters extends ValkyrieWalkingControllerParameters
   {
      private final ValkyrieJointMap jointMap;

      public NormalModelWalkingControllerParameters(ValkyrieJointMap jointMap)
      {
         super(jointMap);
         this.jointMap = jointMap;
      }
      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new TestMomentumOptimizationSettings(jointMap);
      }
   }

   private class HeightForBalanceModel extends ValkyrieRobotModel
   {
      public HeightForBalanceModel(){
         super(RobotTarget.SCS, false);
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

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new TestMomentumOptimizationSettings(jointMap);
      }
   }

   private class TestMomentumOptimizationSettings extends ValkyrieMomentumOptimizationSettings
   {
      private final ValkyrieJointMap jointMap;
      public TestMomentumOptimizationSettings(ValkyrieJointMap jointMap)
      {
         super(jointMap);
         this.jointMap=jointMap;
      }
      @Override
      public Vector3D getLinearMomentumWeight()
      {
         return linearMomentumWeight;
      }
   }
}
