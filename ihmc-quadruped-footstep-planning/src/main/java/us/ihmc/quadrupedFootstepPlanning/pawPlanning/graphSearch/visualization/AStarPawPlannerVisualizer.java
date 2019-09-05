package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.visualization;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawStepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AStarPawPlannerVisualizer implements PawStepPlannerListener
{
   private static final int maxNumberOfChildNodes = 500;
   private static final double childNodeSize = 0.005;
   private static final double nodeSize = 0.007;

   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
   private final YoDouble time = new YoDouble("time", registry);

   private final HashMap<PawNode, List<PawNode>> validChildNodeMap = new HashMap<>();
   private final HashMap<PawNode, List<PawNode>> invalidChildNodeMap = new HashMap<>();

   private final List<YoFramePoint2D> yoValidChildNodes = new ArrayList<>();
   private final List<YoFramePoint2D> yoInValidChildNodes = new ArrayList<>();

   private final YoPawNode currentPawNode = new YoPawNode(registry);

   public AStarPawPlannerVisualizer(PlanarRegionsList regions)
   {
      this(0.1, regions);
   }

   public AStarPawPlannerVisualizer(double playbackRate, PlanarRegionsList regions)
   {
      for (int i = 0; i < maxNumberOfChildNodes; i++)
      {
         YoFramePoint2D validChildNode = new YoFramePoint2D("validChildNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition validChildNodeVis = new YoGraphicPosition("validChildNode" + i, validChildNode, childNodeSize, YoAppearance.Green());

         yoValidChildNodes.add(validChildNode);
         graphicsListRegistry.registerYoGraphic("plannerListener", validChildNodeVis);

         YoFramePoint2D invalidChildNode = new YoFramePoint2D("invalidChildNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition invalidChildNodeVis = new YoGraphicPosition("invalidChildNode" + i, invalidChildNode, childNodeSize, YoAppearance.Red());

         yoInValidChildNodes.add(invalidChildNode);
         graphicsListRegistry.registerYoGraphic("plannerListener", invalidChildNodeVis);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoGraphicPosition otherPaws = new YoGraphicPosition(robotQuadrant.getShortName() + "activeNodePaw", currentPawNode.getYoPosition(robotQuadrant), nodeSize, YoAppearance.Orange());
         graphicsListRegistry.registerYoGraphic("plannerListener", otherPaws);

      }
      YoGraphicPosition nodeVis = new YoGraphicPosition("activeNode", currentPawNode.getMovingYoPosition(), nodeSize, YoAppearance.White());

      graphicsListRegistry.registerYoGraphic("plannerListener", nodeVis);

      YoGraphicReferenceFrame worldViz = new YoGraphicReferenceFrame(ReferenceFrame.getWorldFrame(), registry, false, 0.5);
      graphicsListRegistry.registerYoGraphic("plannerListener", worldViz);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      if (regions != null)
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, regions);
      scs.addStaticLinkGraphics(graphics3DObject);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(playbackRate);
      scs.setMaxBufferSize(64000);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.setGroundVisible(false);
      scs.tickAndUpdate();
   }

   @Override
   public void addNode(PawNode node, PawNode previousNode)
   {
      if (previousNode == null)
         return;

      if (!validChildNodeMap.containsKey(previousNode))
         validChildNodeMap.put(previousNode, new ArrayList<>());

      validChildNodeMap.get(previousNode).add(node);

      currentPawNode.set(previousNode);
   }

   @Override
   public void rejectNode(PawNode rejectedNode, PawNode parentNode, PawStepPlannerNodeRejectionReason reason)
   {
      if(validChildNodeMap.containsKey(parentNode))
         validChildNodeMap.get(parentNode).remove(rejectedNode);

      if (!invalidChildNodeMap.containsKey(parentNode))
         invalidChildNodeMap.put(parentNode, new ArrayList<>());

      invalidChildNodeMap.get(parentNode).add(rejectedNode);
   }

   @Override
   public void rejectNode(PawNode rejectedNode, PawStepPlannerNodeRejectionReason reason)
   {
      // TODO
   }

   @Override
   public void plannerFinished(List<PawNode> plan)
   {

   }

   @Override
   public void reportLowestCostNodeList(List<PawNode> plan)
   {

   }

   @Override
   public void tickAndUpdate()
   {
      yoValidChildNodes.forEach(YoFramePoint2D::setToNaN);

      PawNode currentNode = currentPawNode.getEquivalentNode();

      if (currentNode == null)
         return;


      List<PawNode> validChildNodes = validChildNodeMap.get(currentNode);
      if (validChildNodes != null)
      {
         for (int i = 0; i < validChildNodes.size(); i++)
         {
            PawNode childNode = validChildNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoValidChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }

      List<PawNode> invalidChildNodes = invalidChildNodeMap.get(currentNode);
      if (invalidChildNodes != null)
      {
         for (int i = 0; i < invalidChildNodes.size(); i++)
         {
            PawNode childNode = invalidChildNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoInValidChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }

      scs.setTime(time.getDoubleValue());
      scs.tickAndUpdate();
      time.add(1.0);
   }

   public void showAndSleep(boolean autoplay)
   {
      if (autoplay)
         scs.play();
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }
}
