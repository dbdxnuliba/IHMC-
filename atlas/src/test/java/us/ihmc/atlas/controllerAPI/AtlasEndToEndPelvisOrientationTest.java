package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisOrientationTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndPelvisOrientationTest extends EndToEndPelvisOrientationTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @Test// timeout = 130000
   public void testGoHome() throws SimulationExceededMaximumTimeException
   {
      super.testGoHome();
   }

   @Override
   @Test// timeout = 130000
   public void testSingleTrajectoryPoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @Test// timeout = 300000
   public void testQueue() throws SimulationExceededMaximumTimeException
   {
      super.testQueue();
   }

   @Override
   @Test// timeout = 210000
   public void testWalking() throws SimulationExceededMaximumTimeException
   {
      super.testWalking();
   }

   @Override
   @Test// timeout = 200000
   public void testWalkingAfterTrajectory() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingAfterTrajectory();
   }

   @Override
   @Test// timeout = 170000
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @Test// timeout = 170000
   public void testWalkingWithUserControl() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithUserControl();
   }

   @Override
   @Test// timeout = 150000
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
