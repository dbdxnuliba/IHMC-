package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerCostParametersReadOnly;

public class HeightCost implements FootstepCost
{
   private final BipedFootstepPlannerCostParametersReadOnly costParameters;

   private final LinearHeightCost linearHeightCost;
   private final QuadraticHeightCost quadraticHeightCost;

   public HeightCost(BipedFootstepPlannerCostParametersReadOnly costParameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.costParameters = costParameters;

      linearHeightCost = new LinearHeightCost(costParameters, snapper);
      quadraticHeightCost = new QuadraticHeightCost(costParameters, snapper);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (costParameters.useQuadraticHeightCost())
         return quadraticHeightCost.compute(startNode, endNode);
      else
         return linearHeightCost.compute(startNode, endNode);
   }
}
