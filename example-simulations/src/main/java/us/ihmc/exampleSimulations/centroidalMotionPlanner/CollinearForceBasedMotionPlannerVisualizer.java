package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForceBasedCoMMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.CollinearForcePlannerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CollinearForceBasedMotionPlannerVisualizer
{
   private enum TestMotion
   {
      WALK, RUN, JUMP
   };

   private static final double gravityZ = -9.81;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TestMotion motion = TestMotion.JUMP;
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
   private final BagOfBalls comTrack;
   private final BagOfBalls copTrack;
   private final YoFrameVector groundReactionForce;

   // Foot viz parameters
   private final YoAppearanceRGBColor leftFootAppearance = new YoAppearanceRGBColor(Color.CYAN, 0.5);
   private final YoAppearanceRGBColor rightFootAppearance = new YoAppearanceRGBColor(Color.GREEN, 0.5);
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
   // Contact state viz 
   private final YoAppearanceRGBColor contactStateAppearance = new YoAppearanceRGBColor(Color.BLUE, 0.0);
   // Track viz 
   private final YoAppearanceRGBColor comTrackAppearance = new YoAppearanceRGBColor(Color.WHITE, 0.0);
   private final YoAppearanceRGBColor copTrackAppearance = new YoAppearanceRGBColor(Color.RED, 0.0);

   private final int maxNumberOfContactStatesToVisualize;
   private final RecyclingArrayList<ContactState> contactStates = new RecyclingArrayList<>(ContactState.class);

   public CollinearForceBasedMotionPlannerVisualizer()
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);
      dt = new YoDouble(namePrefix + "VizDT", registry);
      dt.set(0.001);
      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);
      motionPlanner = new CollinearForceBasedCoMMotionPlanner(gravity, registry);
      plannerParameters = new CollinearForcePlannerParameters();
      motionPlanner.initialize(plannerParameters);
      maxNumberOfContactStatesToVisualize = 10; //plannerParameters.getNumberOfContactStatesToPlan();

      graphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList footVizList = new YoGraphicsList("FootVizList");
      for (RobotSide side : RobotSide.values)
      {
         Graphics3DObject footViz = new Graphics3DObject();
         footViz.addExtrudedPolygon(defaultFootPolygonPointsInAnkleFrame, 0.001, side == RobotSide.LEFT ? leftFootAppearance : rightFootAppearance);
         YoFramePose footPose = new YoFramePose(side.getCamelCaseName() + "FootPose", worldFrame, registry);
         currentFootPose.put(side, footPose);
         footVizList.add(new YoGraphicShape(side.getCamelCaseName() + "FootViz", footViz, footPose, 1.0));
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
            YoFramePose footPose = new YoFramePose(side.getCamelCaseName() + "FootPose", worldFrame, registry);
            contactPose.put(side, footPose);
            contactStateVizList.add(new YoGraphicShape("ContactState" + i + side.getCamelCaseName() + "SupportPolygon", footViz, footPose, 1.0));
         }
      }
      graphicsListRegistry.registerYoGraphicsList(contactStateVizList);

      comTrack = new BagOfBalls(100, 0.005, namePrefix + "CoMTrack", comTrackAppearance, registry, graphicsListRegistry);
      copTrack = new BagOfBalls(100, 0.005, namePrefix + "CoPTrack", copTrackAppearance, registry, graphicsListRegistry);
      groundReactionForce = new YoFrameVector(namePrefix + "GroundReactionForce", worldFrame, registry);

      scsParameters = new SimulationConstructionSetParameters();
      Robot robot = new Robot("DummyRobot");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.startOnAThread();
      generateContactStatePlan();
      updateContactStateVisualization();
      yoTime.add(dt);
      scs.tickAndUpdate();
   }

   private YoAppearanceRGBColor getContactStateAppearance(int index)
   {
      return new YoAppearanceRGBColor(Color.getHSBColor((float) index / (float) maxNumberOfContactStatesToVisualize * 255, 80, 80), 0.0);
   }

   private final RecyclingArrayList<Point2D> leftFootstepLocations = new RecyclingArrayList<>(Point2D.class);
   private final RecyclingArrayList<Point2D> rightFootstepLocations = new RecyclingArrayList<>(Point2D.class);
   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final double defaultSupportDurationForJumping = 0.6;
   private final double defaultFlightDurationForJumping = 0.1;

   private void generateContactStatePlan()
   {
      switch (motion)
      {
      case WALK:
         populateAlternatingSteps(5, 0.0, 0.0, 0.3, 0.1, RobotSide.LEFT);
         break;
      case JUMP:
         generateFootstepPlanForJumping(5, 0.0, 0.0, 0.1, 0.3, 0.0);
         break;
      case RUN:
         populateAlternatingSteps(5, 0.0, 0.0, 0.3, 0.1, RobotSide.RIGHT);
         break;
      default:
         throw new RuntimeException("Why you do this ?");
      }
   }

   private void generateSupportPolygon(ConvexPolygon2D supportPolygonToSet, Point2D leftFootLocation, Point2D rightFootLocation)
   {
      supportPolygonToSet.clear();
      for (int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         Point2D supportPolygonVertex = defaultFootPolygonPointsInAnkleFrame.get(i);
         if (leftFootLocation != null)
            supportPolygonToSet.addVertex(supportPolygonVertex.getX() + leftFootLocation.getX(), supportPolygonVertex.getY() + leftFootLocation.getY());
         if (rightFootLocation != null)
            supportPolygonToSet.addVertex(supportPolygonVertex.getX() + rightFootLocation.getX(), supportPolygonVertex.getY() + rightFootLocation.getY());
      }
      supportPolygonToSet.update();
   }

   private void populateAlternatingSteps(int numberOfSteps, double xInitial, double yInitial, double stepLength, double stepWidth, RobotSide startSide)
   {
      leftFootstepLocations.add().set(xInitial, yInitial + stepWidth);
      rightFootstepLocations.add().set(xInitial, yInitial - stepWidth);
      double stepX = xInitial;
      RobotSide side = startSide;
      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         stepX += stepLength / 2.0;
         switch (side)
         {
         case LEFT:
            leftFootstepLocations.add().set(stepX, yInitial + stepWidth);
            break;
         case RIGHT:
            leftFootstepLocations.add().set(stepX, yInitial - stepWidth);
            break;
         }
         side = side.getOppositeSide();
      }
      switch (side)
      {
      case LEFT:
         leftFootstepLocations.add().set(stepX, yInitial + stepWidth);
         break;
      case RIGHT:
         leftFootstepLocations.add().set(stepX, yInitial - stepWidth);
         break;
      }
   }

   private void generateFootstepPlanForWalking(int numberOfSteps, double xInitial, double yInitial, double feetWidth, double xStep, double yStep, RobotSide robotSide)
   {
      leftFootstepLocations.clear();
      rightFootstepLocations.clear();
      double stepX = xInitial;
      double stepY = yInitial;
      for (int i = 0; i < numberOfSteps; i++)
      {
         
      }
   }

   private void generateFootstepPlanForJumping(int numberOfJumps, double xInitial, double yInitial, double feetWidth, double xJump, double yJump)
   {
      leftFootstepLocations.clear();
      rightFootstepLocations.clear();
      for (int i = 0; i < numberOfJumps; i++)
      {
         leftFootstepLocations.add().set(xInitial + i * xJump, yInitial + feetWidth + i * yJump);
         rightFootstepLocations.add().set(xInitial + i * xJump, yInitial - feetWidth + i * yJump);
         leftFootstepLocations.add().set(Double.NaN, Double.NaN);
         rightFootstepLocations.add().set(Double.NaN, Double.NaN);
      }
      leftFootstepLocations.add().set(xInitial + numberOfJumps * xJump, yInitial + feetWidth + numberOfJumps * yJump);
      rightFootstepLocations.add().set(xInitial + numberOfJumps * xJump, yInitial - feetWidth + numberOfJumps * yJump);
   }

   private void updateContactStateVisualization()
   {
      int numberOfContactStatesToVisualize = Math.min(maxNumberOfContactStatesToVisualize, contactStates.size());
      for (int i = 0; i < numberOfContactStatesToVisualize; i++)
      {

      }
   }

   private void runMotionPlanner()
   {
      submitContactStates();
   }

   private void submitContactStates()
   {
      motionPlanner.clearContactStateList();
      for (int i = 0; i < contactStates.size(); i++)
         motionPlanner.appendContactStateToList(contactStates.get(i));
   }

   public static void main(String[] args)
   {
      CollinearForceBasedMotionPlannerVisualizer visualizer = new CollinearForceBasedMotionPlannerVisualizer();
   }
}
