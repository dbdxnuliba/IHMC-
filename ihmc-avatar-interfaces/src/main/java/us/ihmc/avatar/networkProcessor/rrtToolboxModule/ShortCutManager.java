package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ExploringDefinition;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class ShortCutManager extends WholeBodyTrajectoryToolboxManager
{

   public ShortCutManager(ExploringDefinition exploringDefinition, int maximumNumberOfUpdate)
   {
      super(exploringDefinition, maximumNumberOfUpdate);
      // TODO Auto-generated constructor stub
   }
   

   @Override
   public SpatialNode createDesiredNode()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void putDesiredNode(SpatialNode desiredNode)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public boolean hasFail()
   {
      // TODO Auto-generated method stub
      return false;
   }

}
