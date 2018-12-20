package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
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
   @Test// timeout = 340000
   public void testHeightModeSwitchWhileWalking() throws Exception
   {
      super.testHeightModeSwitchWhileWalking();
   }

   @Override
   @Test// timeout = 340000
   public void testHeightUsingMultipleWaypoints() throws Exception
   {
      super.testHeightUsingMultipleWaypoints();
   }

   @Override
   @Test// timeout = 370000
   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {
      super.testHeightUsingMultipleWaypointsWhileWalking();
   }

   @Override
   @Test// timeout = 120000
   public void testMultipleWaypoints() throws Exception
   {
      super.testMultipleWaypoints();
   }

   @Override
   @Test// timeout = 170000
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Override
   @Test// timeout = 380000
   public void testSingleWaypointAndWalk() throws Exception
   {
      super.testSingleWaypointAndWalk();
   }

   @Override
   @Test// timeout = 160000
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Override
   @Test// timeout = 460000
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }

}
