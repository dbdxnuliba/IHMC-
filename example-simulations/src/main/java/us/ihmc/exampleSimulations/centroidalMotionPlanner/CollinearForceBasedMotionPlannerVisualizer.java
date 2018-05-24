package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedCoMMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedPlannerResult;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForcePlannerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CollinearForceBasedMotionPlannerVisualizer
{
   private enum TestMotion
   {
      WALK, RUN, JUMP
   };

   private static final double gravityZ = -9.81;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TestMotion motion = TestMotion.RUN;
   private final YoVariableRegistry registry;
   private final CollinearForceBasedCoMMotionPlanner motionPlanner;
   private final CollinearForcePlannerParameters plannerParameters;

   private final SimulationConstructionSet scs;
   private final SimulationConstructionSetParameters scsParameters;

   private final YoDouble dt;
   private final YoDouble yoTime;
   private final YoGraphicsListRegistry graphicsListRegistry;

   private final List<SideDependentList<YoFramePose>> contactPoses = new ArrayList<>();
   private final SideDependentList<YoFramePose> currentFootPose = new SideDependentList<>();
   private final YoFramePoint comPosition;
   private final YoFramePoint copPosition;
   private final YoFrameVector groundForce;
   private final BagOfBalls comTrack;
   private final BagOfBalls copTrack;
   private final YoGraphicVector groundReactionForce;
   // Foot viz parameters
   private final SideDependentList<YoAppearanceRGBColor> footAppearance = new SideDependentList<>(new YoAppearanceRGBColor(Color.CYAN, 0.5),
                                                                                                  new YoAppearanceRGBColor(Color.GREEN, 0.5));
   private final double ankleToToeX = 0.0725;
   private final double ankleToToeY = 0.0225;
   private final double ankleToMidX = 0.03625;
   private final double ankleToMidY = 0.04;
   private final double ankleToHeelX = 0.0175;
   private final double ankleToHeelY = 0.02;
   private final List<Point2D> defaultFootPolygonPointsInAnkleFrame = Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                                new Point2D(ankleToMidX, -ankleToMidY),
                                                                                new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                                new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                            .collect(Collectors.toList());

   private final RecyclingArrayList<SideDependentList<Point2D>> footstepLocations = new RecyclingArrayList<>(new SideDependentListBuilder());
   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final double defaultSupportDurationForJumping = 0.6;
   private final double defaultFlightDurationForJumping = 0.1;
   private final YoBoolean isSupported;
   // Contact state viz
   private final YoAppearanceRGBColor contactStateAppearance = new YoAppearanceRGBColor(Color.BLUE, 0.0);
   // Track viz 
   private final YoAppearanceRGBColor comTrackAppearance = new YoAppearanceRGBColor(Color.WHITE, 0.0);
   private final YoAppearanceRGBColor copTrackAppearance = new YoAppearanceRGBColor(Color.ORANGE, 0.0);

   private final int maxNumberOfContactStatesToVisualize;
   private final int maxNumberOfContactStatesToSubmit;
   private final RecyclingArrayList<ContactState> contactStates = new RecyclingArrayList<>(ContactState.class);
   private final double forceScalingFactor = 0.001;
   private final ExecutionTimer timer;

   public CollinearForceBasedMotionPlannerVisualizer()
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);
      dt = new YoDouble(namePrefix + "VizDT", registry);
      dt.set(0.01);
      isSupported = new YoBoolean(namePrefix + "IsSupported", registry);
      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      motionPlanner = new CollinearForceBasedCoMMotionPlanner(gravity, registry);
      plannerParameters = new CollinearForcePlannerParameters();
      motionPlanner.initialize(plannerParameters);
      maxNumberOfContactStatesToVisualize = 15; //plannerParameters.getNumberOfContactStatesToPlan();
      maxNumberOfContactStatesToSubmit = plannerParameters.getNumberOfContactStatesToPlan();
      graphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList footVizList = new YoGraphicsList("FootVizList");
      for (RobotSide side : RobotSide.values)
      {
         Graphics3DObject footViz = new Graphics3DObject();
         footViz.addExtrudedPolygon(defaultFootPolygonPointsInAnkleFrame, 0.001, footAppearance.get(side));
         YoFramePose footPose = new YoFramePose(side.getCamelCaseName() + "FootPose", worldFrame, registry);
         currentFootPose.put(side, footPose);
         YoGraphicShape yoGraphic = new YoGraphicShape(side.getCamelCaseName() + "FootViz", footViz, footPose, 1.0);
         footVizList.add(yoGraphic);
      }
      graphicsListRegistry.registerYoGraphicsList(footVizList);
      YoGraphicsList contactStateVizList = new YoGraphicsList("ContactStateViz");
      for (int i = 0; i < maxNumberOfContactStatesToVisualize; i++)
      {
         SideDependentList<YoFramePose> contactPose = new SideDependentList<>();
         contactPoses.add(contactPose);
         for (RobotSide side : RobotSide.values)
         {
            Graphics3DObject footViz = new Graphics3DObject();
            footViz.addExtrudedPolygon(defaultFootPolygonPointsInAnkleFrame, 0.001, getContactStateAppearance(i));
            YoFramePose footPose = new YoFramePose(side.getCamelCaseName() + "FootPose" + i, worldFrame, registry);
            contactPose.put(side, footPose);
            YoGraphicShape yoGraphic = new YoGraphicShape("ContactState" + i + side.getCamelCaseName() + "SupportPolygon", footViz, footPose, 1.0);
            contactStateVizList.add(yoGraphic);
         }
      }
      graphicsListRegistry.registerYoGraphicsList(contactStateVizList);
      comPosition = new YoFramePoint(namePrefix + "CoMPosition", worldFrame, registry);
      copPosition = new YoFramePoint(namePrefix + "CoPPosition", worldFrame, registry);
      groundForce = new YoFrameVector(namePrefix + "GroundForce", worldFrame, registry);
      comTrack = new BagOfBalls(1000, 0.001, namePrefix + "CoMTrack", comTrackAppearance, registry, graphicsListRegistry);
      copTrack = new BagOfBalls(1000, 0.001, namePrefix + "CoPTrack", copTrackAppearance, registry, graphicsListRegistry);
      groundReactionForce = new YoGraphicVector(namePrefix + "GroundReactionForce", copPosition, groundForce, new YoAppearanceRGBColor(Color.RED, 0.5));
      graphicsListRegistry.registerYoGraphic("ForceViz", groundReactionForce);

      timer = new ExecutionTimer(getClass().getSimpleName() + "Timer", registry);
      scsParameters = new SimulationConstructionSetParameters();
      Robot robot = new Robot("DummyRobot");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      //scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.startOnAThread();
      runMotionPlanner();
   }

   private YoAppearanceRGBColor getContactStateAppearance(int index)
   {
      return new YoAppearanceRGBColor(Color.getHSBColor((float) index / (float) maxNumberOfContactStatesToVisualize, 0.8f, 0.8f), 0.75);
   }

   private void generateContactStatePlan()
   {
      switch (motion)
      {
      case WALK:
         generateFootstepPlanForWalking(5, 0.0, 0.0, 0.15, 0.2, 0.0, RobotSide.RIGHT);
         break;
      case JUMP:
         generateFootstepPlanForJumping(5, 0.0, 0.0, 0.15, 0.15, 0.0);
         break;
      case RUN:
         generateFootstepPlanForRunning(6, 0.0, 0.0, 0.15, 0.6, 0.0, RobotSide.RIGHT);
         break;
      default:
         throw new RuntimeException("Why you do this ?");
      }
      generateContactStateFromFootstepPlan();
   }

   private void generateContactStateFromFootstepPlan()
   {
      contactStates.clear();
      for (int i = 0; i < footstepLocations.size(); i++)
      {
         ContactState contactState = contactStates.add();
         contactState.reset();
         Point2D leftFootPos = footstepLocations.get(i).get(RobotSide.LEFT);
         Point2D rightFootPos = footstepLocations.get(i).get(RobotSide.RIGHT);
         generateMinimalVertexSupportPolygon(tempPolygon, leftFootPos, rightFootPos);
         PrintTools.debug(leftFootPos.toString() + " " + rightFootPos.toString() + " " + tempPolygon.getCentroid().toString());
         contactState.setSupportPolygon(tempPolygon);
         if (leftFootPos.containsNaN() && rightFootPos.containsNaN())
         {
            contactState.setDuration(0.15);
         }
         else if (leftFootPos.containsNaN() && !rightFootPos.containsNaN())
         {
            contactState.setDuration(motion == TestMotion.RUN ? 0.3 :0.4);
         }
         else if (rightFootPos.containsNaN() && !leftFootPos.containsNaN())
         {
            contactState.setDuration(motion == TestMotion.RUN ? 0.3 : 0.4);
         }
         else
         {
            if(i != 0 && i != contactStates.size() - 1)
               contactState.setDuration(motion == TestMotion.JUMP ? 0.5 : motion == TestMotion.RUN? 0.3 : 0.2);
            else
               contactState.setDuration(motion == TestMotion.RUN ? 0.3 : 0.4);
         }
      }
   }

   private static final List<Point2D> vertexList = new ArrayList<>();
   static
   {
      for (int i = 0; i < 12; i++)
         vertexList.add(new Point2D());
   }

   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final ConvexPolygon2D tempPolygonForScaling = new ConvexPolygon2D();
   private void generateMinimalVertexSupportPolygon(ConvexPolygon2D supportPolygonToSet, Point2D leftFootLocation, Point2D rightFootLocation)
   {
      supportPolygonToSet.clear();
      int numberOfVertices = 0;
      // Consolidate all the vertices to be added to the support polygon
      for (int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         Point2D supportPolygonVertex = defaultFootPolygonPointsInAnkleFrame.get(i);
         if (!leftFootLocation.containsNaN())
            vertexList.get(numberOfVertices++).set(supportPolygonVertex.getX() + leftFootLocation.getX(),
                                                   supportPolygonVertex.getY() + leftFootLocation.getY());
         if (!rightFootLocation.containsNaN())
            vertexList.get(numberOfVertices++).set(supportPolygonVertex.getX() + rightFootLocation.getX(),
                                                   supportPolygonVertex.getY() + rightFootLocation.getY());
      }
      if(numberOfVertices == 0)
      {
         supportPolygonToSet.clear();
         supportPolygonToSet.update();
         return;
      }
      // Generate the minimal vertex polygon. New gift wrapping algorithm
      // Get the max X max Y element. 
      int candidateVertexIndex = 0;
      for (int i = 1; i < numberOfVertices; i++)
      {
         if (vertexList.get(i).getX() > vertexList.get(candidateVertexIndex).getX())
            candidateVertexIndex = i;
         else if (vertexList.get(i).getX() == vertexList.get(candidateVertexIndex).getX()
               && vertexList.get(i).getY() > vertexList.get(candidateVertexIndex).getY())
            candidateVertexIndex = i;
      }
      // Place the top right vertex at the beginning of list
      Point2D topRightVertex = vertexList.get(candidateVertexIndex);
      Point2D firstVertex = vertexList.get(0);
      vertexList.set(0, topRightVertex);
      vertexList.set(candidateVertexIndex, firstVertex);
      // Start the marching
      for (int i = 1; i < numberOfVertices; i++)
      {
         Point2D lastComputedPoint = vertexList.get(i - 1);
         Point2D candidatePoint = vertexList.get(i);
         // Find the next one 
         for (int j = i + 1; j < numberOfVertices; j++)
         {
            Point2D pointUnderConsideration = vertexList.get(j);
            double det = (pointUnderConsideration.getY() - lastComputedPoint.getY()) * (candidatePoint.getX() - lastComputedPoint.getX())
                  - (pointUnderConsideration.getX() - lastComputedPoint.getX()) * (candidatePoint.getY() - lastComputedPoint.getY());
            boolean swap = det > 0.0 || (det == 0.0 && lastComputedPoint.distance(pointUnderConsideration) > lastComputedPoint.distance(candidatePoint));
            if (swap)
            {
               vertexList.set(j, candidatePoint);
               vertexList.set(i, pointUnderConsideration);
               candidatePoint = pointUnderConsideration;
            }
         }
         double det2 = (topRightVertex.getY() - lastComputedPoint.getY()) * (candidatePoint.getX() - lastComputedPoint.getX())
               - (topRightVertex.getX() - lastComputedPoint.getX()) * (candidatePoint.getY() - lastComputedPoint.getY());
         boolean terminate = det2 > 0.0 || (det2 == 0.0 && lastComputedPoint.distance(candidatePoint) < lastComputedPoint.distance(topRightVertex));
         if (terminate)
         {
            numberOfVertices = i;
            break;
         }
      }
      // Submit the computed vertices to the polygon. Polygon will recompute but that cant be avoided right now
      tempPolygonForScaling.clear();
      tempPolygonForScaling.addVertices(vertexList, numberOfVertices);
      tempPolygonForScaling.update();
      polygonScaler.scaleConvexPolygon(tempPolygonForScaling, 0.005, supportPolygonToSet);
   }

   private void generateFootstepPlanForRunning(int numberOfSteps, double xInitial, double yInitial, double feetWidth, double xStep, double yStep,
                                               RobotSide startSide)
   {
      footstepLocations.clear();
      SideDependentList<Point2D> feetLocations = footstepLocations.add();
      double stepX = xInitial;
      double stepY = yInitial;
      RobotSide side = startSide;
      feetLocations.get(RobotSide.LEFT).set(stepX, stepY + feetWidth / 2.0);
      feetLocations.get(RobotSide.RIGHT).set(stepX, stepY - feetWidth / 2.0);
      feetLocations = footstepLocations.add();
      feetLocations.get(side).set(Double.NaN, Double.NaN);
      feetLocations.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth / 2.0));
      for (int i = 0; i < numberOfSteps; i++)
      {
         feetLocations = footstepLocations.add();
         feetLocations.get(side).set(Double.NaN, Double.NaN);
         feetLocations.get(side.getOppositeSide()).set(Double.NaN, Double.NaN);
         feetLocations = footstepLocations.add();
         stepX += xStep / 2.0;
         stepY += yStep / 2.0;
         feetLocations.get(side).set(stepX, stepY + side.negateIfRightSide(feetWidth / 2.0));
         side = side.getOppositeSide();
         feetLocations.get(side).set(Double.NaN, Double.NaN);
      }
      feetLocations = footstepLocations.add();
      feetLocations.get(side).set(stepX, stepY + side.negateIfRightSide(feetWidth / 2.0));
      feetLocations.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth / 2.0));
   }

   private void generateFootstepPlanForWalking(int numberOfSteps, double xInitial, double yInitial, double feetWidth, double xStep, double yStep,
                                               RobotSide startSide)
   {
      footstepLocations.clear();
      double stepX = xInitial;
      double stepY = yInitial;
      SideDependentList<Point2D> footstepLocation = footstepLocations.add();
      footstepLocation.get(RobotSide.LEFT).set(stepX, stepY + feetWidth / 2.0);
      footstepLocation.get(RobotSide.RIGHT).set(stepX, stepY - feetWidth / 2.0);
      RobotSide side = startSide;
      for (int i = 0; i < numberOfSteps; i++)
      {
         footstepLocation = footstepLocations.add();
         footstepLocation.get(side).set(Double.NaN, Double.NaN);
         footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth / 2.0));
         footstepLocation = footstepLocations.add();
         footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth / 2.0));
         stepX += xStep / 2.0;
         stepY += yStep / 2.0;
         footstepLocation.get(side).set(stepX, stepY + side.negateIfRightSide(feetWidth / 2.0));
         side = side.getOppositeSide();
      }
      footstepLocation = footstepLocations.add();
      footstepLocation.get(side).set(Double.NaN, Double.NaN);
      footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth / 2.0));
      footstepLocation = footstepLocations.add();
      footstepLocation.get(side).set(stepX, stepY + side.negateIfRightSide(feetWidth / 2.0));
      footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth / 2.0));
   }

   private void generateFootstepPlanForJumping(int numberOfJumps, double xInitial, double yInitial, double feetWidth, double xJump, double yJump)
   {
      footstepLocations.clear();
      for (int i = 0; i < numberOfJumps; i++)
      {
         SideDependentList<Point2D> footstepNode1 = footstepLocations.add();
         SideDependentList<Point2D> footstepNode2 = footstepLocations.add();
         for (RobotSide side : RobotSide.values)
         {
            footstepNode1.get(side).set(xInitial + i * xJump, yInitial + side.negateIfRightSide(feetWidth / 2.0) + i * yJump);
            footstepNode2.get(side).set(Double.NaN, Double.NaN);
         }
      }
      SideDependentList<Point2D> footstepNode1 = footstepLocations.add();
      for (RobotSide side : RobotSide.values)
         footstepNode1.get(side).set(xInitial + numberOfJumps * xJump, yInitial + side.negateIfRightSide(feetWidth / 2.0) + numberOfJumps * yJump);
   }

   private void updateContactStateVisualization(int firstFootStepIndex)
   {
      int numberOfContactStatesToVisualize = Math.min(maxNumberOfContactStatesToVisualize, footstepLocations.size() - firstFootStepIndex);
      int i = 0;
      for (; i < firstFootStepIndex; i++)
      {
         for (RobotSide side : RobotSide.values)
            contactPoses.get(i).get(side).setToNaN();
      }
      for (i = firstFootStepIndex; i < numberOfContactStatesToVisualize + firstFootStepIndex; i++)
      {
         SideDependentList<Point2D> feetLocation = footstepLocations.get(i);
         for (RobotSide side : RobotSide.values)
         {
            Point2D footPosition = feetLocation.get(side);
            contactPoses.get(i).get(side).setPosition(footPosition.getX(), footPosition.getY(), i * 0.001);
         }
      }
      for (; i < contactPoses.size(); i++)
      {
         for (RobotSide side : RobotSide.values)
            contactPoses.get(i).get(side).setToNaN();
      }
   }

   private List<ContactState> contactStatesForPlanner = new ArrayList<>();

   private void runMotionPlanner()
   {
      timer.startMeasurement();
      generateContactStatePlan();
      int i = 0;
      //for (int i = 0; i < contactStates.size(); i++)
      {
         populateContactStatesToSubmit(i);
         updateContactStateVisualization(0);
         submitContactStates();
         motionPlanner.runIterations(1);
         CollinearForceBasedPlannerResult sqpSolution = motionPlanner.getSQPSolution();
         timer.stopMeasurement();
         double currentStateDuration = 10.0;
         int contactStateIndex = 0;
         isSupported.set(true);
         double stateEndTime = contactStates.get(contactStateIndex).getDuration();
         for (double t = 0.0; t < currentStateDuration; t += dt.getDoubleValue())
         {
            if(t > stateEndTime)
            {
               contactStateIndex++;
               if(contactStateIndex == contactStates.size())
                  break;
               stateEndTime += contactStates.get(contactStateIndex).getDuration();
               isSupported.set(contactStates.get(contactStateIndex).isSupported());
               updateContactStateVisualization(contactStateIndex);
            }
            sqpSolution.compute(t);
            this.comPosition.set(sqpSolution.getDesiredCoMPosition());
            this.copPosition.set(sqpSolution.getDesiredCoPPosition());
            this.groundForce.set(sqpSolution.getDesiredGroundReactionForce());
            this.groundForce.scale(forceScalingFactor);
            updateCoMCoPVisualization();
            tick();
         }
      }
   }

   private void updateCoMCoPVisualization()
   {
      comTrack.setBallLoop(comPosition);
      copTrack.setBallLoop(copPosition);
      groundReactionForce.update();
   }

   private void tick()
   {
      yoTime.add(dt);
      scs.tickAndUpdate();
   }

   private void populateContactStatesToSubmit(int firstContactStateIndex)
   {
      contactStatesForPlanner.clear();
      int numberOfContactStatesForPlanner = Math.min(maxNumberOfContactStatesToSubmit, contactStates.size() - firstContactStateIndex);
      for (int i = 0; i < numberOfContactStatesForPlanner; i++)
         contactStatesForPlanner.add(contactStates.get(i + firstContactStateIndex));
   }

   private void submitContactStates()
   {
      motionPlanner.clearContactStateList();
      for (int i = 0; i < contactStatesForPlanner.size(); i++)
         motionPlanner.appendContactStateToList(contactStatesForPlanner.get(i));
   }

   private class SideDependentListBuilder extends GenericTypeBuilder<SideDependentList<Point2D>>
   {
      @Override
      public SideDependentList<Point2D> newInstance()
      {
         SideDependentList<Point2D> list = new SideDependentList<Point2D>();
         for (RobotSide side : RobotSide.values)
            list.put(side, new Point2D());
         return list;
      }
   }

   public static void main(String[] args)
   {
      CollinearForceBasedMotionPlannerVisualizer visualizer = new CollinearForceBasedMotionPlannerVisualizer();
   }
}
