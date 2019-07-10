package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.*;
import us.ihmc.atlas.*;
import us.ihmc.avatar.behaviorTests.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.*;

public class AtlasWalkToObject extends AvatarSphereDectionBehaviorTest
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


   @Test
   public void testWalkTOObjectDetected() throws Exception
   {
      super.testWalkToObjectDetected();
   }

}
