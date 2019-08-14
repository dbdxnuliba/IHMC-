package us.ihmc.atlas.networkProcessor.kinematicsStreamingToolboxModule;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Test
   @Override
   public void testFixedGoal() throws SimulationExceededMaximumTimeException
   {
      super.testFixedGoal();
   }

   @Test
   @Override
   public void testFixedGoalOperatorOffset() throws SimulationExceededMaximumTimeException
   {
      super.testFixedGoalOperatorOffset();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public DRCRobotModel newRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }
}
