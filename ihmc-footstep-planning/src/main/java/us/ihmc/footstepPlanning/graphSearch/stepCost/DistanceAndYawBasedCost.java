package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final BipedFootstepPlannerParametersReadOnly parameters;
   private final FootstepPlannerCostParameters costParameters;

   private final EuclideanDistanceAndYawBasedCost euclideanCost;
   private final QuadraticDistanceAndYawCost quadraticCost;

   public DistanceAndYawBasedCost(BipedFootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
      costParameters = parameters.getCostParameters();

      euclideanCost = new EuclideanDistanceAndYawBasedCost(parameters);
      quadraticCost = new QuadraticDistanceAndYawCost(parameters);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (costParameters.useQuadraticDistanceCost())
         return quadraticCost.compute(startNode, endNode);
      else
         return euclideanCost.compute(startNode, endNode);
   }
}
