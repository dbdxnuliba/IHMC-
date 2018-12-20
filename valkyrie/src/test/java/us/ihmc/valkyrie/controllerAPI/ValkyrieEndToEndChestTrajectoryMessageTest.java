package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndChestTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndChestTrajectoryMessageTest extends EndToEndChestTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @Test// timeout = 190000
   public void testLookingLeftAndRight() throws Exception
   {
      super.testLookingLeftAndRight();
   }

   @Override
   @Test// timeout = 200000
   public void testLookingLeftAndRightInVariousTrajectoryFrames() throws Exception
   {
      super.testLookingLeftAndRightInVariousTrajectoryFrames();
   }

   @Override
   @Test// timeout = 200000
   public void testMessageWithALotOfTrajectoryPoints() throws Exception
   {
      super.testMessageWithALotOfTrajectoryPoints();
   }

   @Override
   @Test// timeout = 200000
   public void testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp() throws Exception
   {
      super.testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp();
   }

   @Override
   @Test// timeout = 130000
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @Test// timeout = 250000
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Override
   @Test// timeout = 140000
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Override
   @Test// timeout = 220000
   public void testQueueWithUsingDifferentTrajectoryFrameWithoutOverride() throws Exception
   {
      super.testQueueWithUsingDifferentTrajectoryFrameWithoutOverride();
   }

   @Override
   @Test// timeout = 83000
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   @Test// timeout = 130000
   public void testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint() throws Exception
   {
      super.testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint();
   }

   @Override
   @Test// timeout = 140000
   public void testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint() throws Exception
   {
      super.testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint();
   }

   @Override
   @Test// timeout = 120000
   public void testSettingWeightMatrixUsingSingleTrajectoryPoint() throws Exception
   {
      super.testSettingWeightMatrixUsingSingleTrajectoryPoint();
   }

   @Override
   @Test// timeout = 130000
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @Test// timeout = 190000
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Override
   @Test// timeout = 190000
   public void testStopAllTrajectoryRepeatedly() throws Exception
   {
      super.testStopAllTrajectoryRepeatedly();
   }
}
