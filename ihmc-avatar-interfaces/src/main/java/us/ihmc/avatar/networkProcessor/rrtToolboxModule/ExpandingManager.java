package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class ExpandingManager extends WholeBodyTrajectoryToolboxManager
{

   public ExpandingManager(int maximumNumberOfUpdate)
   {
      super(maximumNumberOfUpdate);
   }

   @Override
   public SpatialNode getDesiredNode()
   {
      return null;
   }

   @Override
   public boolean isDone()
   {
      return false;
   }


}
