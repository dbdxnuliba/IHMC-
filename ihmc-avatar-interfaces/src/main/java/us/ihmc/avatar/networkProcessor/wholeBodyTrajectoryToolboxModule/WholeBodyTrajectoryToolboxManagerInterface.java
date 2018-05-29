package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import us.ihmc.manipulation.planning.exploringSpatial.SpatialNode;

public interface WholeBodyTrajectoryToolboxManagerInterface
{
   abstract SpatialNode createDesiredNode();

   abstract void putDesiredNode(SpatialNode desiredNode);

   abstract boolean isDone();

   abstract boolean hasFail();
}
