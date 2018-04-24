package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class ExpandingManager extends WholeBodyTrajectoryToolboxManager
{
   private List<SpatialNode> validNodes = new ArrayList<SpatialNode>();
   private List<SpatialNode> inValidNodes = new ArrayList<SpatialNode>();

   public ExpandingManager(int maximumNumberOfUpdate)
   {
      super(maximumNumberOfUpdate);
   }

   @Override
   public SpatialNode createDesiredNode()
   {
      return null;
   }

   @Override
   public boolean isDone()
   {
      // TODO : when the progress of the recent added node is 1.
      return false;
   }

   @Override
   public void putDesiredNode(SpatialNode desiredNode)
   {
      
   }

   @Override
   public boolean hasFail()
   {
      return false;
   }


}
