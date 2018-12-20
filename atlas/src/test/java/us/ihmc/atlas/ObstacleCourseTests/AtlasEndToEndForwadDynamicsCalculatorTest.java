package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.AvatarEndToEndForwadDynamicsCalculatorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class AtlasEndToEndForwadDynamicsCalculatorTest extends AvatarEndToEndForwadDynamicsCalculatorTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @Test// timeout = 300000
   public void testStanding() throws Exception
   {
      super.testStanding();
   }

   @Override
   @Test// timeout = 300000
   public void testFloating() throws Exception
   {
      super.testFloating();
   }

   @Override
   @Test// timeout = 300000
   public void testWalking() throws Exception
   {
      super.testWalking();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }
}
