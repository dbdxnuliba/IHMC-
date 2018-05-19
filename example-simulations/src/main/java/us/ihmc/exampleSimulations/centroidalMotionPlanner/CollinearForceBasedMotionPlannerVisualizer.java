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
import us.ihmc.robotics.lists.GenericTypeBuilder;
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
         footViz.addExtrudedPolygon(defaultFootPolygonPointsInAnkleFrame, 0.001, footAppearance.get(side));
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
            YoFramePose footPose = new YoFramePose(side.getCamelCaseName() + "FootPose" + i, worldFrame, registry);
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
      return new YoAppearanceRGBColor(Color.getHSBColor((float)index / (float) maxNumberOfContactStatesToVisualize, 0.8f, 0.8f), 0.0);
   }

   private void generateContactStatePlan()
   {
      switch (motion)
      {
      case WALK:
         generateFootstepPlanForWalking(5, 0.0, 0.0, 0.1, 0.3, 0.0, RobotSide.LEFT);
         break;
      case JUMP:
         generateFootstepPlanForJumping(5, 0.0, 0.0, 0.1, 0.3, 0.0);
         break;
      case RUN:
         //generateFootstepPlanForWalking(5, 0.0, 0.0, 0.3, 0.1, RobotSide.RIGHT);
         //break;
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

   private void generateFootstepPlanForWalking(int numberOfSteps, double xInitial, double yInitial, double feetWidth, double xStep, double yStep,
                                               RobotSide startSide)
   {
      footstepLocations.clear();
      double stepX = xInitial;
      double stepY = yInitial;
      SideDependentList<Point2D> footstepLocation = footstepLocations.add();
      footstepLocation.get(RobotSide.LEFT).set(stepX, stepY + feetWidth);
      footstepLocation.get(RobotSide.RIGHT).set(stepX, stepY - feetWidth);
      RobotSide side = startSide;
      for (int i = 0; i < numberOfSteps; i++)
      {
         footstepLocation = footstepLocations.add();
         footstepLocation.get(side).set(Double.NaN, Double.NaN);
         footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth));
         footstepLocation = footstepLocations.add();
         footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfLeftSide(feetWidth));
         stepX += xStep / 2.0;
         stepY += yStep / 2.0;
         footstepLocation.get(side.getOppositeSide()).set(stepX, stepY + side.negateIfRightSide(feetWidth));
      }
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
            footstepNode1.get(side).set(xInitial + i * xJump, yInitial + side.negateIfRightSide(feetWidth) + i * yJump);
            footstepNode2.get(side).set(Double.NaN, Double.NaN);
         }
      }
      SideDependentList<Point2D> footstepNode1 = footstepLocations.add();
      for (RobotSide side : RobotSide.values)
         footstepNode1.get(side).set(xInitial + numberOfJumps * xJump, yInitial + side.negateIfRightSide(feetWidth) + numberOfJumps * yJump);
   }

   private void updateContactStateVisualization()
   {
      int numberOfContactStatesToVisualize = Math.min(maxNumberOfContactStatesToVisualize, footstepLocations.size());
      for (int i = 0; i < numberOfContactStatesToVisualize; i++)
      {
         SideDependentList<Point2D> feetLocation = footstepLocations.get(i);
         PrintTools.debug("Contact State " + i + ": " + feetLocation.get(RobotSide.LEFT).toString() + " " + feetLocation.get(RobotSide.RIGHT).toString());
         for(RobotSide side: RobotSide.values)
         {
            Point2D footPosition = feetLocation.get(side);
            contactPoses.get(i).get(side).setPosition(footPosition.getX(), footPosition.getY(), 0.0);
         }
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
