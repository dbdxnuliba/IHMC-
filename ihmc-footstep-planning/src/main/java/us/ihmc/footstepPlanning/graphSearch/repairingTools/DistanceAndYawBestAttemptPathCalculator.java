package us.ihmc.footstepPlanning.graphSearch.repairingTools;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.HeuristicsTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;

public class DistanceAndYawBestAttemptPathCalculator implements BestAttemptPathCalculator
{
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;
   private FootstepGraph footstepGraph;

   public DistanceAndYawBestAttemptPathCalculator(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
   }

   @Override
   public void addFootstepGraph(FootstepGraph footstepGraph)
   {
      this.footstepGraph = footstepGraph;
   }

   @Override
   public FootstepGraph getFootstepGraph()
   {
      return footstepGraph;
   }

   @Override
   public double computeRemainingCostToGoal(FootstepNode goalNode, FootstepNode possibleEndNode, FootstepNode childNode)
   {
      Point2D goalPoint = goalNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Point2D midstanceNodePoint = new Point2D(0.5 * (possibleEndNode.getX() + childNode.getX()), 0.5 * (possibleEndNode.getY() + childNode.getY()));
      double midstanceYaw = AngleTools.interpolateAngle(possibleEndNode.getYaw(), childNode.getYaw(), 0.5);

      double referenceYaw = computeReferenceYaw(possibleEndNode, goalNode);
      double distanceToGoal = midstanceNodePoint.distance(goalPoint);
      double yawToGoal = AngleTools.computeAngleDifferenceMinusPiToPi(midstanceYaw, referenceYaw);

      double minSteps = distanceToGoal / parameters.getMaximumStepReach() + Math.abs(yawToGoal) / (0.5 * parameters.getMaximumStepYaw());
      return distanceToGoal + costParameters.getYawWeight() * Math.abs(yawToGoal) + costParameters.getCostPerStep() * minSteps;
   }

   private double computeReferenceYaw(FootstepNode node, FootstepNode goalNode)
   {
      double pathHeading = Math.atan2(goalNode.getY() - node.getY(), goalNode.getX() - node.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      return HeuristicsTools.computeReferenceYaw(node, goalNode, parameters.getFinalTurnProximity(), pathHeading);
   }
}
