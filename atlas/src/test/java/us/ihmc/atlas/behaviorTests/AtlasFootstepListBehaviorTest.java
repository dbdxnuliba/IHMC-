package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCFootstepListBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("atlas")
public class AtlasFootstepListBehaviorTest extends DRCFootstepListBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasFootstepListBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
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

   @Override
   @Test// timeout = 290000
   public void testSideStepping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepping();
   }

   @Override
   @Test// timeout = 170000
   public void testStepLongerThanMaxStepLength() throws SimulationExceededMaximumTimeException
   {
      super.testStepLongerThanMaxStepLength();
   }

   @Override
   @Test// timeout = 360000
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      super.testStop();
   }

   @Override
   @Test// timeout = 270000
   public void testTwoStepsForwards() throws SimulationExceededMaximumTimeException
   {
      super.testTwoStepsForwards();
   }
}
