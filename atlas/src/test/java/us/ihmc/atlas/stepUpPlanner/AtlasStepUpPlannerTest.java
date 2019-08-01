package us.ihmc.atlas.stepUpPlanner;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.stepUpPlanner.AvatarStepUpPlannerTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasStepUpPlannerTest extends AvatarStepUpPlannerTest
{

   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   @Test
   public void testStepUpPlannerLowStep() throws SimulationExceededMaximumTimeException
   {
      super.walkUpToHighStep(0.30);
   }

   @Test
   public void testStepUpPlannerMediumStep() throws SimulationExceededMaximumTimeException
   {
      super.walkUpToHighStep(0.35);
   }

   @Test
   public void testStepUpPlannerHighStep() throws SimulationExceededMaximumTimeException
   {
      super.walkUpToHighStep(0.40);
   }

}
