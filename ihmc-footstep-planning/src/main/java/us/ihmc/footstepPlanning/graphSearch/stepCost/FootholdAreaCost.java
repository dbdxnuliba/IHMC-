package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootholdAreaCost implements FootstepCost
{
   private static final double weight = 2.0;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepNodeSnapperReadOnly snapper;

   public FootholdAreaCost(SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapperReadOnly snapper)
   {
      this.footPolygons = footPolygons;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      FootstepNodeSnapData snapData = snapper.getSnapData(endNode);
      ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(endNode.getRobotSide()).getArea();

      if (!footholdAfterSnap.isEmpty())
      {
         double percentAreaUnoccupied = 1.0 - area / footArea;
         return percentAreaUnoccupied * weight;
      }

      return 0.0;
   }
}
