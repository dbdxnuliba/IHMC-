package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerCostParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class EuclideanBasedCost implements FootstepCost
{
   private final BipedFootstepPlannerParametersReadOnly parameters;
   private final BipedFootstepPlannerCostParametersReadOnly costParameters;

   public EuclideanBasedCost(BipedFootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode) + costParameters.getCostPerStep();
   }
}
