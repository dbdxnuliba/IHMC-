package us.ihmc.footstepPlanning.graphSearch.repairingTools;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepEdge;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.Set;

public interface BestSubOptimalPathCalculator
{
   default FootstepNode computeBestEndNode(SideDependentList<FootstepNode> goalNodes, Set<FootstepNode> expandedNodes, HashMap<FootstepNode, FootstepEdge> incomingBestEdges)
   {
      FootstepNode bestNode = null;
      double lowestCost = Double.POSITIVE_INFINITY;

      for (FootstepNode expandedNode : expandedNodes)
      {
         FootstepEdge edgeOfExpandedNode = incomingBestEdges.get(expandedNode);
         if (edgeOfExpandedNode == null)
            continue;

         FootstepNode childOfExpandedNode = edgeOfExpandedNode.getStartNode();
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

   double computeRemainingCostToGoal(FootstepNode goalNodes, FootstepNode possibleEndNode, FootstepNode childNode);
}
