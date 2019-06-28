package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.AvatarKinematicsSimulation;

public class AtlasKinematicsSimulation
{
   public static void main(String[] args)
   {
      AvatarKinematicsSimulation.createForManualTest(new AtlasRobotModel(AtlasBehaviorModule.ATLAS_VERSION, RobotTarget.SCS, false), false);
   }
}
