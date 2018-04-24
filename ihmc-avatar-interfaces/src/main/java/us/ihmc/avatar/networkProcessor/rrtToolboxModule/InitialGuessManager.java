package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ExploringDefinition;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class InitialGuessManager extends WholeBodyTrajectoryToolboxManager
{
   private ExploringDefinition exploringDefinition;
   
   private int terminalConditionNumberOfValidNodes;
   private int numberOfValidNodes;

   private List<SpatialNode> initialGuesses;

   public InitialGuessManager(ExploringDefinition exploringDefinition, int maximumNumberOfUpdate, int terminalConditionNumberOfValidNodes)
   {
      super(maximumNumberOfUpdate);
      this.exploringDefinition = exploringDefinition;
      this.terminalConditionNumberOfValidNodes = terminalConditionNumberOfValidNodes;
      this.numberOfValidNodes = 0;
      this.initialGuesses = new ArrayList<SpatialNode>();
   }

   public void addInitialGuess(SpatialNode node)
   {
      initialGuesses.add(node);  
   }
   
   @Override
   public void initialize()
   {
      super.initialize();
      this.numberOfValidNodes = 0;
      this.initialGuesses = new ArrayList<SpatialNode>();
   }

   @Override
   public SpatialNode createRandomNode()
   {
      SpatialData randomSpatialData = exploringDefinition.getRandomSpatialData();
      SpatialNode node = new SpatialNode(randomSpatialData);
      
      return node;
   }

   @Override
   public boolean isDone()
   {
      return isExceedMaximumNumberOfUpdate() || numberOfValidNodes >= terminalConditionNumberOfValidNodes;
   }

}
