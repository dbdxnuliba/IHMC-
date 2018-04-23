package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;

public class InitialGuessManager extends WholeBodyTrajectoryToolboxManager
{
   private int terminalConditionNumberOfValidNodes;
   private int numberOfValidNodes;
   
   public InitialGuessManager(int maximumNumberOfUpdate, int terminalConditionNumberOfValidNodes)
   {
      super(maximumNumberOfUpdate);
      this.terminalConditionNumberOfValidNodes = terminalConditionNumberOfValidNodes;
      this.numberOfValidNodes = 0;
   }
   
   @Override
   public void initialize()
   {
      this.numberOfValidNodes = 0;
      super.initialize();

   }

   @Override
   public SpatialNode getDesiredNode()
   {
      return null;
   }

   @Override
   public boolean isDone()
   {
      return isExceedMaximumNumberOfUpdate() || numberOfValidNodes >= terminalConditionNumberOfValidNodes;
   }




}
