package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public interface WholeBodyTrajectoryToolboxManagerInterface
{
   abstract void initialize();
   
   abstract SpatialNode getDesiredNode();

   abstract void update();
   
   abstract boolean isDone();
}
