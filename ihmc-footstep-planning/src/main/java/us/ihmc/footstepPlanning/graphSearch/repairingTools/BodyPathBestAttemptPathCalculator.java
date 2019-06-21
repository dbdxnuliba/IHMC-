package us.ihmc.footstepPlanning.graphSearch.repairingTools;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.HeuristicsTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.AngleTools;

public class BodyPathBestAttemptPathCalculator implements BestAttemptPathCalculator
{
   private static final double pathViolationWeight = 30.0;
   private final BodyPathPlanner bodyPath;
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;
   private FootstepGraph footstepGraph;

   private double goalAlpha = 1.0;

   public BodyPathBestAttemptPathCalculator(FootstepPlannerParameters parameters, BodyPathPlanner bodyPath)
   {
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
      this.bodyPath = bodyPath;
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
      Point2D midstanceNodePoint = new Point2D(0.5 * (possibleEndNode.getX() + childNode.getX()), 0.5 * (possibleEndNode.getY() + childNode.getY()));
      double midstanceYaw = AngleTools.interpolateAngle(possibleEndNode.getYaw(), childNode.getYaw(), 0.5);
      Pose2D closestPointOnPath = new Pose2D();

      double alpha = bodyPath.getClosestPoint(midstanceNodePoint, closestPointOnPath);
      alpha = MathTools.clamp(alpha, 0.0, goalAlpha);
      bodyPath.getPointAlongPath(alpha, closestPointOnPath);

      double distanceToPath = closestPointOnPath.getPosition().distance(midstanceNodePoint);
      double pathLength = bodyPath.computePathLength(alpha) - bodyPath.computePathLength(goalAlpha);
      double remainingDistance = pathLength + pathViolationWeight * distanceToPath;

      double referenceYaw = HeuristicsTools.computeReferenceYaw(possibleEndNode, goalNode, parameters.getFinalTurnProximity(), closestPointOnPath.getYaw());

      double yawError = AngleTools.computeAngleDifferenceMinusPiToPi(midstanceYaw, referenceYaw);
      double yawCost = costParameters.getYawWeight() * pathViolationWeight * Math.abs(yawError);
      double minSteps = remainingDistance / parameters.getMaximumStepReach();
      return remainingDistance + yawCost + costParameters.getCostPerStep() * minSteps;
   }

   public void setGoalAlpha(double alpha)
   {
      goalAlpha = alpha;
   }
}
