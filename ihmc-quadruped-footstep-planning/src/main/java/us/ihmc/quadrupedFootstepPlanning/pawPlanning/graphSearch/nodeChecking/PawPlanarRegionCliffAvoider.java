package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawCliffDetectionTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawPlanarRegionCliffAvoider extends PawNodeChecker
{
   private final PawStepPlannerParametersReadOnly parameters;
   private final PawNodeSnapperReadOnly snapper;

   private PawNode startNode;

   public PawPlanarRegionCliffAvoider(PawStepPlannerParametersReadOnly parameters, PawNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void addStartNode(PawNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransform)
   {
      this.startNode = startNode;
   }

   @Override
   public boolean isNodeValidInternal(PawNode node)
   {
      if (startNode != null && startNode.equals(node))
         return true;

      if (!hasPlanarRegions())
         return true;

      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      int xIndex = node.getXIndex(movingQuadrant);
      int yIndex = node.getYIndex(movingQuadrant);
      RigidBodyTransform pawTransformToWorld = new RigidBodyTransform();
      PawNodeTools.getSnappedNodeTransformToWorld(xIndex, yIndex, snapper.getSnapData(xIndex, yIndex).getSnapTransform(), pawTransformToWorld);

      Point3D pawInWorld = new Point3D();
      pawTransformToWorld.transform(pawInWorld);

      double forward = movingQuadrant.isQuadrantInFront() ?
            parameters.getMinimumFrontEndForwardDistanceFromCliffBottoms() :
            parameters.getMinimumHindEndForwardDistanceFromCliffBottoms();
      double backward = movingQuadrant.isQuadrantInFront() ?
            -parameters.getMinimumFrontEndBackwardDistanceFromCliffBottoms() :
            -parameters.getMinimumHindEndBackwardDistanceFromCliffBottoms();
      double left = parameters.getMinimumLateralDistanceFromCliffBottoms();
      double right = -parameters.getMinimumLateralDistanceFromCliffBottoms();

      boolean isNearCliff = PawCliffDetectionTools.isNearCliff(planarRegionsList, pawInWorld, node.getStepYaw(), parameters, forward, backward, left, right);

      if (isNearCliff)
      {
         rejectNode(node, PawStepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
         return false;
      }

      return true;
   }
}
