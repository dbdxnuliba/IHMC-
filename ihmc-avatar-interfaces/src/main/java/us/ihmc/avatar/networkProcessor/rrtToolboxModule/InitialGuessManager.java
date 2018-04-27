package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ExploringDefinition;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class InitialGuessManager extends WholeBodyTrajectoryToolboxManager
{
   private int terminalConditionNumberOfValidNodes;

   private List<SpatialNode> initialGuesses;

   public InitialGuessManager(ExploringDefinition exploringDefinition, int maximumNumberOfUpdate, int terminalConditionNumberOfValidNodes)
   {
      super(exploringDefinition, maximumNumberOfUpdate);
      this.terminalConditionNumberOfValidNodes = terminalConditionNumberOfValidNodes;
      this.initialGuesses = new ArrayList<SpatialNode>();
   }
   
   public List<SpatialNode> getValidInitialGuesses()
   {
      return initialGuesses;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      this.initialGuesses = new ArrayList<SpatialNode>();
   }

   @Override
   public SpatialNode createDesiredNode()
   {
      SpatialData randomSpatialData = exploringDefinition.getRandomSpatialData();
      SpatialNode desiredNode = new SpatialNode(randomSpatialData);
      return desiredNode;
   }

   @Override
   public boolean isDone()
   {
      return isExceedMaximumNumberOfUpdate() || initialGuesses.size() >= terminalConditionNumberOfValidNodes;
   }

   @Override
   public void putDesiredNode(SpatialNode desiredNode)
   {
      if(desiredNode.isValid())
      {
         initialGuesses.add(desiredNode);
      }
   }

   @Override
   public boolean hasFail()
   {
      if(initialGuesses.size() < 1)
         return true;
      else
         return false;
   }
}
