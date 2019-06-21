package us.ihmc.footstepPlanning.graphSearch.repairingTools;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.NodeComparator;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class BestAttemptHeuristicPathCalculator
{
   private final NodeComparator nodeComparator;

   public BestAttemptHeuristicPathCalculator(NodeComparator nodeComparator)
   {
      this.nodeComparator = nodeComparator;
   }

   public FootstepNode computeBestEndNode(Set<FootstepNode> expandedNodes)
   {
      List<FootstepNode> sortedNodes = new ArrayList<>(expandedNodes);
      sortedNodes.sort(nodeComparator);

      return sortedNodes.get(0);
   }
}
