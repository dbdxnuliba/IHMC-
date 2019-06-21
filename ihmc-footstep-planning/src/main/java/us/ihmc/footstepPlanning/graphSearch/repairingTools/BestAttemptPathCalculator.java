package us.ihmc.footstepPlanning.graphSearch.repairingTools;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.Set;

public interface BestAttemptPathCalculator
{
   default FootstepNode computeBestEndNode(SideDependentList<FootstepNode> goalNodes, Set<FootstepNode> expandedNodes)
   {
      FootstepNode bestNode = null;
      double lowestCost = Double.POSITIVE_INFINITY;

      for (FootstepNode expandedNode : expandedNodes)
      {
         FootstepNode childOfExpandedNode = getParentNode(expandedNode);
         if (childOfExpandedNode == null)
            continue;

         double nodeCost = computeRemainingCostToGoal(goalNodes, expandedNode, childOfExpandedNode);

         if (nodeCost < lowestCost)
         {
            bestNode = expandedNode;
            lowestCost = nodeCost;
         }
      }

      return bestNode;
   }

   default double computeRemainingCostToGoal(SideDependentList<FootstepNode> goalNodes, FootstepNode possibleEndNode, FootstepNode childNode)
   {
      double lowerCost = Double.POSITIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values)
      {
         lowerCost = Math.min(lowerCost, computeRemainingCostToGoal(goalNodes.get(robotSide), possibleEndNode, childNode));
      }

      return lowerCost;
   }

   default FootstepNode getParentNode(FootstepNode node)
   {
      return getFootstepGraph().getParentNode(node);
   }

   default List<FootstepNode> getBestAttemptPath(SideDependentList<FootstepNode> goalNodes, Set<FootstepNode> expandedNodes)
   {
      FootstepNode bestEndNode = computeBestEndNode(goalNodes, expandedNodes);
      return getFootstepGraph().getPathFromStart(bestEndNode);
   }

   void addFootstepGraph(FootstepGraph footstepGraph);

   FootstepGraph getFootstepGraph();

   double computeRemainingCostToGoal(FootstepNode goalNodes, FootstepNode possibleEndNode, FootstepNode childNode);
}
