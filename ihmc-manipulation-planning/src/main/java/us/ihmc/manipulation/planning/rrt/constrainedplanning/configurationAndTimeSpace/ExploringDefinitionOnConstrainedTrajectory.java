package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;

public class ExploringDefinitionOnConstrainedTrajectory extends ExploringDefinition
{
   private double trajectoryTime = 0.0;

   public ExploringDefinitionOnConstrainedTrajectory(List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                                                     List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      super(endEffectorTrajectories, explorationConfigurations);

      for (int i = 0; i < endEffectorTrajectories.size(); i++)
      {
         WaypointBasedTrajectoryCommand trajectoryCommand = endEffectorTrajectories.get(i);
         if (trajectoryTime < trajectoryCommand.getLastWaypointTime())
            trajectoryTime = trajectoryCommand.getLastWaypointTime();
      }

      PrintTools.info("trajectoryTime is " + trajectoryTime);
   }

   @Override
   public double getExploringProgress(SpatialNode node)
   {
      if (trajectoryTime > 0.0)
         return node.getTime() / trajectoryTime;
      else
         return -1.0;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }
}
