package us.ihmc.footstepPlanning.flatGroundPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.NodeComparator;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.*;

public class AStarAnimation
{
   private static final double footLength = 0.22;
   private static final double footWidth = 0.11;

   private static final double heightAboveField = 0.75;
   private static final double childNodeSize = 0.02;
   private static final double highestCostInflation = 1.5;
   private static final double lowestCostDeflation = 0.5;

   private static final Point3D startPosition = new Point3D();
   private static final Point3D goalPosition = new Point3D(1.5, 0.0, 0.0);

   private enum StepInViz {EXPANSION, SNAPPING, FILTER_VALID, FILTER_STEEP, FILTER_GOOD, FILTER_AREA, FILTER_CLIFFS, SCORE, SCORE_HEURISTIC }

   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble time = new YoDouble("Time", registry);

   private final YoEnum<StepInViz> stepEnum = new YoEnum<>("stepInViz", registry, StepInViz.class);

   private final PlanarRegionsList planarRegionsList;

   private final FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters()
   {
      @Override
      public double getMaximumStepYaw()
      {
         return 0.01;
      }
      @Override
      public double getMinimumStepYaw()
      {
         return -0.01;
      }

      @Override
      public double getMaximumStepReach()
      {
         return 0.6;
      }

      @Override
      public double getMaximumStepWidth()
      {
         return 0.5;
      }


      @Override
      public double getMinimumStepLength()
      {
         return -getMaximumStepReach();
      }

      @Override
      public double getCliffHeightToAvoid()
      {
         return 0.3;
      }

      @Override
      public double getMinimumDistanceFromCliffBottoms()
      {
         return 0.1;
      }

      @Override
      public double getMinimumSurfaceInclineRadians()
      {
         return Math.toRadians(30.0);
      }
   };

   private final SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();

   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepNodeSnapper nodeSnapper;
   private final FootstepNodeSnapper nodeWiggler;
   private final FootstepGraph graph;

   private final SnapBasedCheckerComponent validSnapChecker;
   private final SnapBasedCheckerComponent inclineChecker;
   private final SnapBasedCheckerComponent goodPositionChecker;
   private final SnapBasedCheckerComponent enoughAreaChecker;
   private final FootstepNodeChecker cliffChecker;

   private final FootstepNodeCheckerOfCheckers checker;

   private final BagOfBalls pathViz;
   private final BagOfBalls childNodeViz;
   private final BagOfBalls snappedNodeViz;

   private final FootstepCost footstepCostEvaluator;
   private final CostToGoHeuristics costToGoHeuristics;

   private final HashSet<FootstepNode> expandedNodes = new HashSet<>();

   private final List<YoGraphicPosition> scoredNodesViz = new ArrayList<>();
   private final List<YoGraphicPosition> heuristicScoredNodesViz = new ArrayList<>();

   private final FootstepNode startNode;
   private FootstepNode endNode;
   private final SideDependentList<FootstepNode> goalNodes = new SideDependentList<>();
   private final PriorityQueue<FootstepNode> stack;

   public AStarAnimation()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D footPolygon = new ConvexPolygon2D();
         footPolygon.addVertex(0.5 * footLength, 0.5 * footWidth);
         footPolygon.addVertex(0.5 * footLength, -0.5 * footWidth);
         footPolygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
         footPolygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
         footPolygon.update();
         footPolygons.put(robotSide, footPolygon);
      }

      int numberOfNodes = 500;


      pathViz = new BagOfBalls(500, 0.05, "Path", YoAppearance.Green(), GraphicType.BALL, registry, graphicsListRegistry);
      childNodeViz = new BagOfBalls(numberOfNodes, childNodeSize, "ChildNodes", YoAppearance.Blue(), GraphicType.BALL, registry, graphicsListRegistry);
      snappedNodeViz = new BagOfBalls(numberOfNodes, childNodeSize, "SnappedChildNodes", YoAppearance.Red(), GraphicType.BALL, registry, graphicsListRegistry);

      for (int i = 0; i < numberOfNodes; i++)
      {
         YoFramePoint3D locatoin = new YoFramePoint3D("scoredNode" + i, "", ReferenceFrame.getWorldFrame(), registry);
         YoFramePoint3D heuristicLocatoin = new YoFramePoint3D("heuristicScoredNode" + i, "", ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition graphicPosition = new YoGraphicPosition("scoredNode" + i, locatoin, childNodeSize, YoAppearance.Red(), GraphicType.BALL);
         YoGraphicPosition heuristicGraphicPosition = new YoGraphicPosition("heuristicScoredNode" + i, heuristicLocatoin, childNodeSize, YoAppearance.Red(), GraphicType.BALL);
         scoredNodesViz.add(graphicPosition);
         heuristicScoredNodesViz.add(heuristicGraphicPosition);
         graphicsListRegistry.registerYoGraphic("Scored Nodes", graphicPosition);
         graphicsListRegistry.registerYoGraphic("Heuristic Scored Nodes", heuristicGraphicPosition);
      }

      graph = new FootstepGraph();
      nodeExpansion = new ParameterBasedNodeExpansion(parameters);
      nodeSnapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      nodeWiggler = new FootstepNodeSnapAndWiggler(footPolygons, parameters);

      validSnapChecker = new ValidSnapChecker(nodeSnapper);
      inclineChecker = new SurfaceInclineSnapChecker(parameters, nodeSnapper);
      goodPositionChecker = new GoodFootstepPositionChecker(parameters, nodeSnapper);
      enoughAreaChecker = new EnoughAreaSnapChecker(parameters, footPolygons, nodeSnapper);
      cliffChecker = new PlanarRegionBaseOfCliffAvoider(parameters,  nodeSnapper, footPolygons);

      checker = new FootstepNodeCheckerOfCheckers(Arrays.asList(new SnapBasedNodeChecker(parameters, footPolygons, nodeSnapper), cliffChecker));

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setSnapper(nodeSnapper);
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setIncludeAreaCost(true);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);

      footstepCostEvaluator = costBuilder.buildCost();
      costToGoHeuristics = new DistanceAndYawBasedHeuristics(parameters.getCostParameters().getAStarHeuristicsWeight(), parameters);

      PlanarRegionEnvironmentInterface environmentInterface = new AnimationEnvironment();

      planarRegionsList = environmentInterface.getPlanarRegionsList();


      nodeSnapper.setPlanarRegions(planarRegionsList);
      nodeWiggler.setPlanarRegions(planarRegionsList);
      cliffChecker.setPlanarRegions(planarRegionsList);
      checker.setPlanarRegions(planarRegionsList);

      startNode = new FootstepNode(startPosition.getX(), startPosition.getY());
      nodeSnapper.snapFootstepNode(startNode);

      ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", ReferenceFrame.getWorldFrame());
      ((PoseReferenceFrame) goalFrame).setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), goalPosition));
      for (RobotSide side : RobotSide.values)
      {
         FramePose3D goalNodePose = new FramePose3D(goalFrame);
         goalNodePose.setY(side.negateIfRightSide(parameters.getIdealFootstepWidth() / 2.0));
         goalNodePose.changeFrame(ReferenceFrame.getWorldFrame());
         FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
         nodeSnapper.snapFootstepNode(goalNode);
         goalNodes.put(side, goalNode);
      }

      YoFramePoint3D leftGoalNodePositoin = new YoFramePoint3D("leftGoalNodePosition", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D rightGoalNodePositoin = new YoFramePoint3D("rightGoalNodePosition", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition goalLeft = new YoGraphicPosition("leftGoalNode", leftGoalNodePositoin, 0.05,  YoAppearance.DarkRed(), GraphicType.BALL);
      YoGraphicPosition goalRight= new YoGraphicPosition("rightGoalNode", rightGoalNodePositoin, 0.05,  YoAppearance.DarkRed(), GraphicType.BALL);
      graphicsListRegistry.registerYoGraphic("Goal Nodes", goalLeft);
      graphicsListRegistry.registerYoGraphic("Goal Nodes", goalRight);

      leftGoalNodePositoin.set(goalNodes.get(RobotSide.LEFT).getX(), goalNodes.get(RobotSide.LEFT).getY(), 0.0);
      rightGoalNodePositoin.set(goalNodes.get(RobotSide.RIGHT).getX(), goalNodes.get(RobotSide.RIGHT).getY(), 0.0);

      graph.initialize(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNodes, costToGoHeuristics);
      stack = new PriorityQueue<>(nodeComparator);

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(environmentInterface.getTerrainObject3D().getLinkGraphics());
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setMaxBufferSize(64000);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.tickAndUpdate();
   }

   public void run()
   {
      scs.startOnAThread();

      stack.add(startNode);
      int iteration = 0;
      while (!stack.isEmpty())
      {
         iteration++;

         FootstepNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;

         if (checkAndHandleNodeAtGoal(nodeToExpand))
            break;

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         for (FootstepNode neighbor : neighbors)
         {
            // Checks if the footstep (center of the foot) is on a planar region
            if (!checker.isNodeValid(neighbor, nodeToExpand))
            {
               continue;
            }

            double cost = footstepCostEvaluator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);

            if (!parameters.getReturnBestEffortPlan() || endNode == null || stack.comparator().compare(neighbor, endNode) < 0)
               stack.add(neighbor);
         }

         runVizOnNode(nodeToExpand);
      }

      ThreadTools.sleepForever();
   }

   private boolean checkAndHandleNodeAtGoal(FootstepNode nodeToExpand)
   {
      RobotSide nodeSide = nodeToExpand.getRobotSide();
      if (goalNodes.get(nodeSide).equals(nodeToExpand))
      {
         endNode = goalNodes.get(nodeSide.getOppositeSide());
         graph.checkAndSetEdge(nodeToExpand, endNode, 0.0);
         return true;
      }

      return false;
   }

   private void runVizOnNode(FootstepNode nodeToExpand)
   {

      expandedNodes.add(nodeToExpand);
      updatePathViz(nodeToExpand);

     tickAndUpdate();

      HashSet<FootstepNode> childNodes = nodeExpansion.expandNode(nodeToExpand);
      stepEnum.set(StepInViz.EXPANSION);
      updateChildNodeViz(childNodes);
      tickAndUpdate();

      stepEnum.set(StepInViz.SNAPPING);

      snapNodesAndUpdateViz(childNodes);
      tickAndUpdate();

      stepEnum.set(StepInViz.FILTER_VALID);

//      List<FootstepNode> filteredNodes = filterAll(nodeToExpand, childNodes);
      List<FootstepNode> filteredNodes = filterValidSnaps(nodeToExpand, childNodes);
      tickAndUpdate();


      stepEnum.set(StepInViz.FILTER_STEEP);

      filteredNodes = filterSteepInclines(nodeToExpand, filteredNodes);
      tickAndUpdate();


      stepEnum.set(StepInViz.FILTER_GOOD);

      filteredNodes = filterGoodPositions(nodeToExpand, filteredNodes);
      tickAndUpdate();

      stepEnum.set(StepInViz.FILTER_AREA);


      filteredNodes = filterEnoughArea(nodeToExpand, filteredNodes);
      tickAndUpdate();


      stepEnum.set(StepInViz.FILTER_CLIFFS);

      filteredNodes = filterCliffs(nodeToExpand, filteredNodes);
      tickAndUpdate();

      stepEnum.set(StepInViz.SCORE);

      scoreTheTransitions(nodeToExpand, filteredNodes);
      tickAndUpdate();

      stepEnum.set(StepInViz.SCORE_HEURISTIC);

      scoreTheTransitionsWithHeuristic(nodeToExpand, filteredNodes);
      tickAndUpdate();
   }

   private void updatePathViz(FootstepNode endNode)
   {
      pathViz.reset();
      List<FootstepNode> pathNodes = graph.getPathFromStart(endNode);
      for (int i = 0; i < pathNodes.size(); i++)
      {
         FootstepNode pathNode = pathNodes.get(i);
         FootstepNodeSnapData snapData = nodeSnapper.snapFootstepNode(pathNode);
         Point3D pointInWorld = new Point3D(pathNode.getX(), pathNode.getY(), 0.0);
         snapData.getSnapTransform().transform(pointInWorld);
         pathViz.setBall(new FramePoint3D(ReferenceFrame.getWorldFrame(), pointInWorld), i);
      }
   }

   private void updateChildNodeViz(HashSet<FootstepNode> childNodes)
   {
      for (YoGraphicPosition graphicPosition : heuristicScoredNodesViz)
         graphicPosition.setPositionToNaN();

      childNodeViz.reset();
      int counter = 0;
      for (FootstepNode childNode : childNodes)
      {
         FramePoint3D childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), heightAboveField);
         childNodeViz.setBall(childPoint, counter);
         counter++;
      }
   }

   private void snapNodesAndUpdateViz(HashSet<FootstepNode> childNodes)
   {
      snappedNodeViz.reset();
      childNodeViz.reset();
      // snap everything
      int counter = 0;
      for (FootstepNode childNode : childNodes)
      {
         FootstepNodeSnapData snapData = nodeSnapper.snapFootstepNode(childNode);
         FramePoint3D childPoint;
         if (snapData.getSnapTransform().containsNaN())
         {
            childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), 0.0);
         }
         else
         {
            childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), 0.0);
            snapData.getSnapTransform().transform(childPoint);
         }
         snappedNodeViz.setBall(childPoint, counter);
         counter++;
      }
   }

   private List<FootstepNode> filterValidSnaps(FootstepNode previousNode, HashSet<FootstepNode> childNodes)
   {
      return filterWithChecker(validSnapChecker, previousNode, childNodes);
   }

   private List<FootstepNode> filterSteepInclines(FootstepNode previousNode, List<FootstepNode> childNodes)
   {
      return filterWithChecker(inclineChecker, previousNode, childNodes);
   }

   private List<FootstepNode> filterGoodPositions(FootstepNode previousNode, List<FootstepNode> childNodes)
   {
      return filterWithChecker(goodPositionChecker, previousNode, childNodes);
   }

   private List<FootstepNode> filterEnoughArea(FootstepNode previousNode, List<FootstepNode> childNodes)
   {
      return filterWithChecker(enoughAreaChecker, previousNode, childNodes);
   }


   private List<FootstepNode> filterWithChecker(SnapBasedCheckerComponent checker, FootstepNode previousNode, Collection<FootstepNode> childNodes)
   {
      List<FootstepNode> filteredNodes = new ArrayList<>();

      snappedNodeViz.reset();
      // snap everything
      int counter = 0;
      for (FootstepNode childNode : childNodes)
      {
         if (checker.isNodeValid(childNode, previousNode))
         {
            FootstepNodeSnapData snapData = nodeSnapper.snapFootstepNode(childNode);
            FramePoint3D childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), 0.0);
            snapData.getSnapTransform().transform(childPoint);
            snappedNodeViz.setBall(childPoint, counter);
            counter++;
            filteredNodes.add(childNode);
         }
      }

      return filteredNodes;
   }

   private List<FootstepNode> filterCliffs(FootstepNode previousNode, List<FootstepNode> childNodes)
   {
      List<FootstepNode> filteredNodes = new ArrayList<>();

      snappedNodeViz.reset();
      // snap everything
      int counter = 0;
      for (FootstepNode childNode : childNodes)
      {
         if (cliffChecker.isNodeValid(childNode, previousNode))
         {
            FootstepNodeSnapData snapData = nodeSnapper.snapFootstepNode(childNode);
            FramePoint3D childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), 0.0);
            snapData.getSnapTransform().transform(childPoint);
            snappedNodeViz.setBall(childPoint, counter);
            counter++;
            filteredNodes.add(childNode);
         }
      }

      return filteredNodes;
   }


   private void scoreTheTransitions(FootstepNode previousNode, List<FootstepNode> childNodes)
   {
      double highestCost = Double.NEGATIVE_INFINITY;
      double lowestCost = Double.POSITIVE_INFINITY;
      for (FootstepNode childNode : childNodes)
      {
         double cost = footstepCostEvaluator.compute(previousNode, childNode);
         lowestCost = Math.min(cost, lowestCost);
         highestCost = Math.max(cost, highestCost);
      }

      snappedNodeViz.reset();
      for (int nodeNumber = 0; nodeNumber < childNodes.size(); nodeNumber++)
      {
         FootstepNode childNode = childNodes.get(nodeNumber);
         double cost = footstepCostEvaluator.compute(previousNode, childNode);
         double fractionAlongCostSpectrum = (cost - lowestCost) / (highestCost - lowestCost);
         double nodeSizeScaling = InterpolationTools.linearInterpolate(lowestCostDeflation, highestCostInflation, fractionAlongCostSpectrum);

         FootstepNodeSnapData snapData = nodeSnapper.snapFootstepNode(childNode);
         FramePoint3D childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), 0.0);
         snapData.getSnapTransform().transform(childPoint);
         scoredNodesViz.get(nodeNumber).setPosition(childPoint);
         scoredNodesViz.get(nodeNumber).setGlobalScaleProvider(() -> nodeSizeScaling);
      }
   }

   private void scoreTheTransitionsWithHeuristic(FootstepNode previousNode, List<FootstepNode> childNodes)
   {
      for (YoGraphicPosition graphicPosition : scoredNodesViz)
         graphicPosition.setPositionToNaN();
      for (YoGraphicPosition graphicPosition : heuristicScoredNodesViz)
         graphicPosition.setPositionToNaN();

      double highestCost = Double.NEGATIVE_INFINITY;
      double lowestCost = Double.POSITIVE_INFINITY;
      for (FootstepNode childNode : childNodes)
      {
         double cost = footstepCostEvaluator.compute(previousNode, childNode) +  getCheapestCostToGo(costToGoHeuristics, childNode, goalNodes);
         lowestCost = Math.min(cost, lowestCost);
         highestCost = Math.max(cost, highestCost);
      }



      for (int nodeNumber = 0; nodeNumber < childNodes.size(); nodeNumber++)
      {
         FootstepNode childNode = childNodes.get(nodeNumber);
         double cost = footstepCostEvaluator.compute(previousNode, childNode) +  getCheapestCostToGo(costToGoHeuristics, childNode, goalNodes);
         double fractionAlongCostSpectrum = (cost - lowestCost) / (highestCost - lowestCost);

         double nodeSizeScaling = InterpolationTools.linearInterpolate(lowestCostDeflation, highestCostInflation , fractionAlongCostSpectrum);

         FootstepNodeSnapData snapData = nodeSnapper.snapFootstepNode(childNode);
         FramePoint3D childPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), childNode.getX(), childNode.getY(), 0.0);
         snapData.getSnapTransform().transform(childPoint);
         heuristicScoredNodesViz.get(nodeNumber).setPosition(childPoint);
         heuristicScoredNodesViz.get(nodeNumber).setGlobalScaleProvider(() -> nodeSizeScaling);
      }
   }

   private static double getCheapestCostToGo(CostToGoHeuristics costToGoHeuristics, FootstepNode node, SideDependentList<FootstepNode> goalNodes)
   {
      return Math.min(costToGoHeuristics.compute(node, goalNodes.get(RobotSide.LEFT)), costToGoHeuristics.compute(node, goalNodes.get(RobotSide.RIGHT)));
   }

   public void tickAndUpdate()
   {
      scs.setTime(time.getDoubleValue());
      scs.tickAndUpdate();
      time.add(1.0);
   }

   public static void main(String[] args)
   {
      AStarAnimation animation = new AStarAnimation();
      animation.run();
   }
}
