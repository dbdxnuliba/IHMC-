package us.ihmc.valkyrie.heightForBalance;

import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.heightForBalanceTest.AvatarHeightForBalanceTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.awt.*;

public class ValkyrieHeightForBalanceTest extends AvatarHeightForBalanceTest
{
   private final RobotTarget target = RobotTarget.SCS;
   private ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS,false);

   /**
    *  Almost all tests do better in maximum recoverable push than the 'normal' configuration, except for two tests, which seem to have
    *  a similar problem.
    */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushStanding() throws Exception
   {
      // No-height max recoverable percentWeight: 0.55
      percentWeight = 0.58;
      super.testPushStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushContraDiagonalInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.98
      percentWeight = 1.07;
      super.testPushContraDiagonalInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushContraDiagonalFrontalInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 1.23
      percentWeight = 1.30;
      super.testPushContraDiagonalFrontalInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushDiagonalInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.76
      percentWeight = 0.81;
      super.testPushDiagonalInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushDiagonalFrontalInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 1.25
      percentWeight = 1.26;
      super.testPushDiagonalFrontalInSwing();
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
      percentWeight = 0.90;
      super.testPushBackInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushRightInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.62
      percentWeight = 0.62;                            // Needs improvement: DISTANCE case
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
   @Test()
   public void testIterativePush() throws Exception
   {
      // No-height max recoverable percentWeight: 0.54
      percentWeight = 0.0;
      super.testIterativePush();
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
}
