package us.ihmc.avatar.collinearForcePlanner;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner.ContactStatePlanGenerator;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.TransformHelperTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GraphicsUpdatablePlaybackListener;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalDynamicsRobot;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalRobotController;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalRobotPhysicalProperties;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class CollinearForcePlannerDynamicsVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum Motion
   {
      WALK, RUN, JUMP
   };

   private final int maxNumberOfContactStatesToVisualize = 15;
   private final String namePrefix = "robot";
   private final YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
   private final YoVariableRegistry graphicsSubRegistry = new YoVariableRegistry("GraphicsRegistry");
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final SideDependentList<YoFramePose> solePose = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> feetSupportPolygon = new SideDependentList<>();
   private final SideDependentList<Vector3D> soleToFootFrameOffset = new SideDependentList<>();
   private final ArrayList<YoGraphicPolygon3D> contactStateViz = new ArrayList<>();
   private final ArrayList<YoFramePose> contactStateLabels = new ArrayList<>();
   private final List<ContactState> contactStatePlan = new ArrayList<>();
   private final ContactStatePlanGenerator contactStatePlanner;
   private final Robot robot;
   private final CollinearForceVisualizationController robotController;
   private final SimulationConstructionSet scs;
   private final YoDouble dt = new YoDouble("dT", registry);
   private final YoDouble yoTime;
   
   public CollinearForcePlannerDynamicsVisualizer(DRCRobotModel robotModel, DRCRobotJointMap jointMap)
   {
      RobotDescription robotDescription = robotModel.getRobotDescription();
      getFootSupportPolygonAndCreateSoleFrames(jointMap, robotDescription);
      createFeetGraphics(jointMap, robotDescription);
      contactStatePlanner = createContactStatePlanner();
      PlaybackListener contactStatePlaybackListener = createContactStateGraphics();

      robotController = getOrCreateRobotController();
      robot = getOrCreateCentroidalDynamicsRobot(robotController);
      scs = createAndSetupSCS(robot, contactStatePlaybackListener);
      yoTime = robot.getYoTime();
      robotController.setControllerTime(yoTime);
      initialize();
   }

   private CollinearForceVisualizationController getOrCreateRobotController()
   {
      if (robotController != null)
         return robotController;
      CollinearForceVisualizationController controller = new CollinearForceVisualizationController(solePose, feetSupportPolygon, graphicsListRegistry);
      controller.setupController(18.0, new FrameVector3D(worldFrame, 0.0, 0.0, -9.81));
      return controller;
   }

   private Robot getOrCreateCentroidalDynamicsRobot(CentroidalRobotController robotController)
   {
      if (robot != null)
         return robot;
      CentroidalRobotPhysicalProperties centroidalProperties = new CentroidalRobotPhysicalProperties()
      {
         @Override
         public double getNominalHeight()
         {
            return 0.435;
         }

         @Override
         public double getMass()
         {
            return 18.0;
         }

         @Override
         public DenseMatrix64F getInertia()
         {
            double Ixx = 0.1, Ixy = 0.0, Ixz = 0.0, Iyy = 0.1, Iyz = 0.0, Izz = 0.1;
            DenseMatrix64F inertiaTensor = new DenseMatrix64F(3, 3, true, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
            return inertiaTensor;
         }

         @Override
         public SideDependentList<ConvexPolygon2D> getDefaultSupportPolygons()
         {
            return feetSupportPolygon;
         }
      };
      CentroidalDynamicsRobot robotModel = new CentroidalDynamicsRobot("", centroidalProperties);
      FloatingRootJointRobot robot = robotModel.addControllerAndCreateRobot(robotController, graphicsListRegistry);
      return robot;
   }

   private ContactStatePlanGenerator createContactStatePlanner()
   {
      int numberOfVertices = feetSupportPolygon.get(RobotSide.LEFT).getNumberOfVertices() + feetSupportPolygon.get(RobotSide.RIGHT).getNumberOfVertices();
      return new ContactStatePlanGenerator(numberOfVertices, 1e-5);
   }

   private SimulationConstructionSet createAndSetupSCS(Robot robot, PlaybackListener contactStatePlaybackListener)
   {
      dt.set(0.0004);
      int buffersize = 1024 * 256;
      PrintTools.debug("Buffer size: " + buffersize);
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(buffersize);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      registry.addChild(graphicsSubRegistry);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.attachPlaybackListener(contactStatePlaybackListener);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.startOnAThread();
      return scs;
   }

   private PlaybackListener createContactStateGraphics()
   {
      YoGraphicsList contactStateGraphicsList = new YoGraphicsList("ContactStates");
      int maxNumberOfVertices = feetSupportPolygon.get(RobotSide.LEFT).getNumberOfVertices() + feetSupportPolygon.get(RobotSide.RIGHT).getNumberOfVertices();
      AppearanceDefinition textAppearance = new YoAppearanceRGBColor(Color.BLACK, 0.0);
      for (int i = 0; i < maxNumberOfContactStatesToVisualize; i++)
      {
         AppearanceDefinition contactStateAppearance = new YoAppearanceRGBColor(Color.getHSBColor((float) i / maxNumberOfContactStatesToVisualize, 0.8f, 0.8f),
                                                                                0.0);
         YoGraphicPolygon3D contactState = new YoGraphicPolygon3D("contactState" + i + "SupportPolygon", maxNumberOfVertices, 0.01, contactStateAppearance,
                                                                  graphicsSubRegistry);
         Graphics3DObject object = new Graphics3DObject();
         object.rotate(Math.PI / 2, Axis.X);
         object.addText(i + "", 0.001, textAppearance);
         YoFramePose textPose = new YoFramePose("TextPose" + i, worldFrame, graphicsSubRegistry);
         YoGraphicShape contactStateName = new YoGraphicShape("contactState" + i + "Number", object, textPose, 0.05);
         contactStateGraphicsList.add(contactState);
         contactStateGraphicsList.add(contactStateName);
         contactStateViz.add(contactState);
         contactStateLabels.add(textPose);
      }
      graphicsListRegistry.registerYoGraphicsList(contactStateGraphicsList);
      return new GraphicsUpdatablePlaybackListener(contactStateViz);
   }

   private void getFootSupportPolygonAndCreateSoleFrames(DRCRobotJointMap jointMap, RobotDescription robotDescription)
   {
      for (RobotSide side : RobotSide.values)
      {
         String sideName = side.getCamelCaseNameForMiddleOfExpression();
         YoFramePose soleFramePose = new YoFramePose(namePrefix + sideName + "SolePose", worldFrame, registry);
         solePose.put(side, soleFramePose);
         String jointBeforeFootName = jointMap.getJointBeforeFootName(side);
         JointDescription jointBeforeFoot = robotDescription.getJointDescription(jointBeforeFootName);
         ArrayList<GroundContactPointDescription> groundContactPointDescription = jointBeforeFoot.getGroundContactPoints();
         Vector3D footToSoleOffsetVector = new Vector3D();
         ConvexPolygon2D footSupportPolygon = new ConvexPolygon2D();
         if (groundContactPointDescription == null)
            throw new RuntimeException("Unable to get ground contact points for " + sideName + " foot");
         for (GroundContactPointDescription description : groundContactPointDescription)
         {
            Vector3D offsetFromJoint = description.getOffsetFromJoint();
            footSupportPolygon.addVertex(offsetFromJoint.getX(), offsetFromJoint.getY());
            footToSoleOffsetVector.add(offsetFromJoint);
         }
         footToSoleOffsetVector.scale(-1.0 / groundContactPointDescription.size());
         footSupportPolygon.update();
         footSupportPolygon.translate(footToSoleOffsetVector.getX(), footToSoleOffsetVector.getY());
         soleToFootFrameOffset.put(side, footToSoleOffsetVector);
         feetSupportPolygon.put(side, footSupportPolygon);
      }
   }

   private void createFeetGraphics(DRCRobotJointMap jointMap, RobotDescription robotDescription)
   {
      YoGraphicsList footGraphicsList = new YoGraphicsList("FootGraphics");
      ArtifactList footArtifactsList = new ArtifactList("FootPlotterGraphics");
      for (RobotSide side : RobotSide.values)
      {
         String sideName = side.getCamelCaseNameForMiddleOfExpression();
         YoFramePose footPose = new YoFramePose(namePrefix + sideName + "FootPose", worldFrame, graphicsSubRegistry);
         String jointBeforeFootName = jointMap.getJointBeforeFootName(side);
         LinkDescription footLinkDescription = robotDescription.getLinkDescription(jointBeforeFootName);
         if (footLinkDescription == null)
            throw new RuntimeException("Unable to find " + side.toString() + " foot link");
         LinkGraphicsDescription footGraphicsDescription = footLinkDescription.getLinkGraphics();
         Graphics3DObject footGraphicObject = new Graphics3DObject(footGraphicsDescription.getGraphics3DInstructions());
         YoGraphicShape footGraphic = new YoGraphicShape(namePrefix + sideName + "FootViz", footGraphicObject, footPose.getPosition(),
                                                         footPose.getOrientation(), 1.0);
         YoFramePose soleFramePose = solePose.get(side);
         YoGraphicCoordinateSystem footFrameGraphic = new YoGraphicCoordinateSystem(side.getCamelCaseName() + "FootFrameGraphic", soleFramePose, 0.1);
         soleFramePose.attachVariableChangedListener(new VariableChangedListener()
         {
            FramePose3D tempPose = new FramePose3D();
            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               soleFramePose.getFramePose(tempPose);
               tempPose.appendTranslation(soleToFootFrameOffset.get(side));
               footPose.set(tempPose);
            }
         });
         footGraphicsList.add(footGraphic);
         footGraphicsList.add(footFrameGraphic);
      }
      graphicsListRegistry.registerArtifactList(footArtifactsList);
      graphicsListRegistry.registerYoGraphicsList(footGraphicsList);
   }

   private void initialize()
   {
      for (RobotSide side : RobotSide.values)
         solePose.get(side).setPosition(0.0, side.negateIfRightSide(0.1), 0.0);
   }

   private final RecyclingArrayList<FramePoint3D> framePointList = new RecyclingArrayList<>(FramePoint3D.class);
   private final FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private final FramePose3D tempPose = new FramePose3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint2D tempPoint2D = new FramePoint2D();

   public void run()
   {
      BlockingSimulationRunner simulationRunner = new BlockingSimulationRunner(scs, 100000);
      prepareContactStatePlan(Motion.JUMP);
      List<ContactState> contactStatePlanForController = new ArrayList<>();
      // warmup();
      runPlanner();
      robotController.setTimeForStateChange(yoTime.getDoubleValue());
      for (int i = 0; i < contactStatePlan.size(); i++)
      {
         contactStatePlanForController.clear();
         for (int j = i; j < contactStatePlan.size(); j++)
            contactStatePlanForController.add(contactStatePlan.get(j));
         updateContactStateVisualization(contactStatePlanForController);
         ContactState currentState = contactStatePlanForController.get(0);
         try
         {
            simulationRunner.simulateAndBlockAndCatchExceptions(currentState.getDuration());
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void warmup()
   {
      for (int i = 0; i < 1000; i++)
         runPlanner();
   }
   
   private void runPlanner()
   {
      robotController.submitContactStateList(contactStatePlan);
      for (int i = 0; i < 2; i++)
      {
         robotController.runIteration();
         scs.tickAndUpdate(i * dt.getDoubleValue());
      }
   }

   private void updateContactStateVisualization(List<ContactState> contactStatesToVisualize)
   {
      int i = 0;
      int numberOfContactStatesToViz = Math.min(contactStatesToVisualize.size(), maxNumberOfContactStatesToVisualize);
      for (i = 0; i < numberOfContactStatesToViz; i++)
      {
         framePointList.clear();
         ContactState contactState = contactStatesToVisualize.get(i);
         contactState.getSupportPolygon(worldFrame, tempPolygon);
         // Setting some Z here to improve the visualization 
         for (int j = 0; j < tempPolygon.getNumberOfVertices(); j++)
            framePointList.add().set(tempPolygon.getVertex(j), 0.0001);
         contactStateViz.get(i).set(framePointList);
         if (tempPolygon.getNumberOfVertices() > 0)
         {
            tempPose.setIncludingFrame(contactState.getPose());
            tempPose.changeFrame(worldFrame);
            contactState.getSupportPolygonCentroid(tempPoint);
            tempPose.setPosition(tempPoint.getX(), tempPoint.getY(), tempPoint.getZ() + 0.001);
            contactStateLabels.get(i).set(tempPose);
         }
         else
            contactStateLabels.get(i).setToNaN();
      }
      for (; i < maxNumberOfContactStatesToVisualize; i++)
      {
         contactStateViz.get(i).setToNaN();
         contactStateLabels.get(i).setToNaN();
      }
   }

   private void prepareContactStatePlan(Motion motion)
   {
      switch (motion)
      {
      case WALK:
         prepareWalkingContactStatePlan();
         break;
      case JUMP:
         prepareJumpingContactStatePlan();
         break;
      case RUN:
         prepareRunningContactStatePlan();
         break;
      default:
         throw new RuntimeException("Invalid motion type");
      }
   }

   private void prepareRunningContactStatePlan()
   {
      int numberOfSteps = 2;
      createContactStates(numberOfSteps * 2 + 1);
      Point2D stepSize = new Point2D(0.8, 0.);
      solePose.get(RobotSide.LEFT).getFramePose(tempPose);
      FramePose2D leftSolePose = new FramePose2D(tempPose);
      solePose.get(RobotSide.RIGHT).getFramePose(tempPose);
      FramePose2D rightSolePose = new FramePose2D(tempPose);
      contactStatePlanner.generateContactStatePlanForRunning(contactStatePlan, numberOfSteps, leftSolePose, rightSolePose, stepSize, RobotSide.LEFT, 0.2, 0.4,
                                                             0.5, 0.5, true, feetSupportPolygon.get(RobotSide.LEFT), feetSupportPolygon.get(RobotSide.RIGHT));
   }

   private void prepareWalkingContactStatePlan()
   {
      int numberOfSteps = 3;
      createContactStates(numberOfSteps * 2 + 1);
      Point2D stepSize = new Point2D(0.4, 0.0);
      solePose.get(RobotSide.LEFT).getFramePose(tempPose);
      FramePose2D leftSolePose = new FramePose2D(tempPose);
      solePose.get(RobotSide.RIGHT).getFramePose(tempPose);
      FramePose2D rightSolePose = new FramePose2D(tempPose);
      contactStatePlanner.generateContactStatePlanForWalking(contactStatePlan, numberOfSteps, leftSolePose, rightSolePose, stepSize, RobotSide.LEFT, 0.4, 0.2,
                                                             0.3, 0.3, true, feetSupportPolygon.get(RobotSide.LEFT), feetSupportPolygon.get(RobotSide.RIGHT));
   }

   private void createContactStates(int numberOfContactStates)
   {
      contactStatePlan.clear();
      for (int i = 0; i < numberOfContactStates; i++)
         contactStatePlan.add(new ContactState());
   }

   private void prepareJumpingContactStatePlan()
   {
      int numberOfJumps = 1;
      createContactStates(numberOfJumps * 2 + 1);
      FramePose2D pelvisPose = new FramePose2D();
      pelvisPose.setPosition((solePose.get(RobotSide.LEFT).getX() + solePose.get(RobotSide.RIGHT).getX()) / 2.0,
                             (solePose.get(RobotSide.LEFT).getY() + solePose.get(RobotSide.RIGHT).getY()) / 2.0);
      Pose2D pelvisPoseChangePerJump = new Pose2D(0.4, 0.0, 0.0);
      Pose2D leftAnklePoseOffset = new Pose2D(0.0, 0.1, 0.0);
      Pose2D rightAnklePoseOffset = new Pose2D(0.0, -0.1, 0.0);
      contactStatePlanner.generateContactStatePlanForJumping(contactStatePlan, numberOfJumps, pelvisPose, pelvisPoseChangePerJump, leftAnklePoseOffset,
                                                             rightAnklePoseOffset, 0.15, 0.50, feetSupportPolygon.get(RobotSide.LEFT),
                                                             feetSupportPolygon.get(RobotSide.RIGHT));
   }
}