package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.manipulation.planning.rrt.exploringSpatial.SpatialNode;

public interface WholeBodyTrajectoryToolboxManagerInterface
{
   abstract SpatialNode createDesiredNode();

   abstract void putDesiredNode(SpatialNode desiredNode);

   abstract boolean isDone();

   abstract boolean hasFail();
}
