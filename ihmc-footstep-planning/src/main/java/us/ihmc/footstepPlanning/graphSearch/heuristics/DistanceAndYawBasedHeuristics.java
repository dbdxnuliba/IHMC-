package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;

   public DistanceAndYawBasedHeuristics(DoubleProvider weight, FootstepPlannerParameters parameters)
   {
      super(weight);
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      Point2D goalPoint = goalNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Point2D nodeMidFootPoint = node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      double euclideanDistance = nodeMidFootPoint.distance(goalPoint);

      double referenceYaw = computeReferenceYaw(node, goalNode);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), referenceYaw);

      double minSteps = euclideanDistance / parameters.getMaximumStepReach() + Math.abs(yaw) / (0.5 * parameters.getMaximumStepYaw());
      return euclideanDistance + costParameters.getYawWeight() * Math.abs(yaw) + costParameters.getCostPerStep() * minSteps;
   }

   private double computeReferenceYaw(FootstepNode node, FootstepNode goalNode)
   {
      double pathHeading = Math.atan2(goalNode.getY() - node.getY(), goalNode.getX() - node.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      return HeuristicsTools.computeReferenceYaw(node, goalNode, parameters.getFinalTurnProximity(), pathHeading);
   }
}
