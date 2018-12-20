package us.ihmc.atlas;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.IHMCROSAPIPacketTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;


@Disabled
public class AtlasIHMCROSAPIPacketTest extends IHMCROSAPIPacketTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test// timeout = 420000
   public void testFuzzyPacketsUsingRos()
   {
      super.testFuzzyPacketsUsingRos();
   }

   @Override
   @Test// timeout = 420000
   public void testFuzzyPacketsWithoutRos()
   {
      super.testFuzzyPacketsWithoutRos();
   }

}
