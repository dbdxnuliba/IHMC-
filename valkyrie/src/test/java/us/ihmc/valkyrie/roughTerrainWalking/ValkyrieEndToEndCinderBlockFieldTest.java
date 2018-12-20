package us.ihmc.valkyrie.roughTerrainWalking;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.EndToEndCinderBlockFieldTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Tag("video")
public class ValkyrieEndToEndCinderBlockFieldTest extends EndToEndCinderBlockFieldTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public double getPelvisOffsetHeight()
   {
      return 0.055;
   }

   @Override
   public double getStepHeightOffset()
   {
      return 0.02;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @Test// timeout = 1100000
   public void testWalkingOverCinderBlockField() throws Exception
   {
      super.testWalkingOverCinderBlockField();
   }
}
