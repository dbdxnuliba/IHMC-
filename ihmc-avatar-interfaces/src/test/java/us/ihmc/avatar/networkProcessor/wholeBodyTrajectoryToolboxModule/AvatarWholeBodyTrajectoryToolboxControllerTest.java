package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.fail;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.ROLL;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.IntStream;

import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AvatarWholeBodyTrajectoryToolboxControllerTest implements MultiRobotTestInterface
{
   protected static final boolean VERBOSE = false;
   private static final boolean GHOSTINGREEN = true;

   private static final AppearanceDefinition ghostApperance = YoAppearance.DarkGreen();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected static final boolean visualize = simulationTestingParameters.getCreateGUI();
   protected static final boolean visualizeManifold = true;

   static
   {
      simulationTestingParameters.setKeepSCSUp(true);
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private StatusMessageOutputManager statusOutputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private WholeBodyTrajectoryToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;

   protected SimulationConstructionSet scs;

   private HumanoidFloatingRootJointRobot robot;
   private HumanoidFloatingRootJointRobot ghost;
   private RobotController toolboxUpdater;

   private WholeBodyTrajectoryToolboxCommandConverter commandConversionHelper;
   private KinematicsToolboxOutputConverter converter;

   private SideDependentList<FramePose3D> endeffectorFramePose = new SideDependentList<>();
   private SideDependentList<YoFramePoseUsingYawPitchRoll> yoEndeffectorPose = new SideDependentList<>();
   private SideDependentList<YoGraphicCoordinateSystem> yoEndeffectorViz = new SideDependentList<>();

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   private static final double TRACKING_TRAJECTORY_POSITION_ERROR_THRESHOLD = 0.05;
   private static final double TRACKING_TRAJECTORY_ORIENTATION_ERROR_THRESHOLD = 0.05;

   @Before
   public void setup()
   {
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel desiredFullRobotModel = createFullRobotModelAtInitialConfiguration();
      commandInputManager = new CommandInputManager(WholeBodyTrajectoryToolboxModule.supportedCommands());
      commandConversionHelper = new WholeBodyTrajectoryToolboxCommandConverter(desiredFullRobotModel);
      commandInputManager.registerConversionHelper(commandConversionHelper);
      commandInputManager.registerMessageUnpacker(WholeBodyTrajectoryToolboxMessage.class,
                                                  MessageUnpackingTools.createWholeBodyTrajectoryToolboxMessageUnpacker());

      converter = new KinematicsToolboxOutputConverter(robotModel);

      statusOutputManager = new StatusMessageOutputManager(WholeBodyTrajectoryToolboxModule.supportedStatus());

      toolboxController = new WholeBodyTrajectoryToolboxController(getRobotModel(), desiredFullRobotModel, commandInputManager, statusOutputManager,
                                                                   mainRegistry, yoGraphicsListRegistry, visualize);

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      if (GHOSTINGREEN)
         KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostApperance);
      ghost = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      ghost.setDynamic(false);
      ghost.setGravity(0);
      hideGhost();

      if (visualize)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            endeffectorFramePose.put(robotSide, new FramePose3D());
            yoEndeffectorPose.put(robotSide,
                                  new YoFramePoseUsingYawPitchRoll("" + robotSide + "yoEndeffectorPose", ReferenceFrame.getWorldFrame(), mainRegistry));
            yoEndeffectorViz.put(robotSide, new YoGraphicCoordinateSystem("" + robotSide + "yoEndeffectorViz", yoEndeffectorPose.get(robotSide), 0.2));
            yoGraphicsListRegistry.registerYoGraphic("" + robotSide + "EndeffectorViz", yoEndeffectorViz.get(robotSide));
         }

         scs = new SimulationConstructionSet(new Robot[] {robot, ghost}, simulationTestingParameters);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
      }
   }

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(-100.0, -100.0, -100.0));
      ghost.update();
   }

   private void hideRobot()
   {
      robot.setPositionInWorld(new Point3D(-100.0, -100.0, -100.0));
      robot.update();
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      if (visualize)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyTransform transformToWorldFrame = fullHumanoidRobotModel.getHandControlFrame(robotSide).getTransformToWorldFrame();

            endeffectorFramePose.get(robotSide).set(transformToWorldFrame);
            yoEndeffectorPose.get(robotSide).set(endeffectorFramePose.get(robotSide));
         }
      }

      new JointAnglesWriter(ghost, fullHumanoidRobotModel).updateRobotConfigurationBasedOnFullRobotModel();
   }

   @After
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (mainRegistry != null)
      {
         mainRegistry.closeAndDispose();
         mainRegistry = null;
      }

      initializationSucceeded = null;

      yoGraphicsListRegistry = null;

      commandInputManager = null;
      statusOutputManager = null;

      toolboxController = null;

      robot = null;
      toolboxUpdater = null;

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   public void testOneBigCircle() throws Exception, UnreasonableAccelerationException
   {
      // Trajectory parameters
      double trajectoryTime = 10.0;
      double circleRadius = 0.5; // Valkyrie, enable ypr, is available for 0.5 radius.
      Point3D circleCenter = new Point3D(0.6, 0.1, 1.0);
      Quaternion circleOrientation = new Quaternion();
      circleOrientation.appendYawRotation(Math.PI * 0.0);
      Quaternion handOrientation = new Quaternion(circleOrientation);

      // WBT toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message, exploration message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == RobotSide.LEFT)
         {
            RigidBody hand = fullRobotModel.getHand(robotSide);

            boolean ccw;
            if (robotSide == RobotSide.RIGHT)
               ccw = false;
            else
               ccw = true;
            FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenter,
                                                                                                      circleOrientation, handOrientation, ccw, 0.0);

            SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
            selectionMatrix.resetSelection();
            selectionMatrix.clearAngularSelection();
            WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime,
                                                                                                                       timeResolution, handFunction,
                                                                                                                       selectionMatrix);

            RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
            MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
            trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
            trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

            handTrajectories.add(trajectory);
            ConfigurationSpaceName[] spaces = {};
            RigidBodyExplorationConfigurationMessage rigidBodyConfiguration = HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces);

            rigidBodyConfigurations.add(rigidBodyConfiguration);

            // manifold
            ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createGoalManifoldMessage(hand, handFunction, trajectoryTime, spaces);
            reachingManifolds.add(reachingManifold);
         }
      }

      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);

      // run toolbox
      runTest(message, 100000);
   }


   protected WholeBodyTrajectoryToolboxMessage createReachingWholeBodyTrajectoryToolboxMessage(FullHumanoidRobotModel fullRobotModel, RigidBody hand,
                                                                                             RobotSide robotSide,
                                                                                             List<ReachingManifoldMessage> reachingManifoldMessages)
   {
      if (VERBOSE)
      {
         OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();
         for (int i = 0; i < oneDoFJoints.length; i++)
            PrintTools.info("" + oneDoFJoints[i].getName() + " " + oneDoFJoints[i].getJointLimitUpper() + " " + oneDoFJoints[i].getJointLimitLower());
      }
      // input
      double extrapolateRatio = 1.5;
      double trajectoryTimeBeforeExtrapolated = 5.0;
      double trajectoryTime = trajectoryTimeBeforeExtrapolated * extrapolateRatio;

      // wbt toolbox configuration message
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      RigidBodyTransform handTransform = handControlFrame.getTransformToWorldFrame();

      RigidBodyTransform closestPointOnManifold = new RigidBodyTransform();
      RigidBodyTransform endTransformOnTrajectory = new RigidBodyTransform();

      List<ReachingManifoldCommand> manifolds = new ArrayList<>();
      for (int i = 0; i < reachingManifoldMessages.size(); i++)
      {
         ReachingManifoldCommand manifold = new ReachingManifoldCommand();
         manifold.setFromMessage(reachingManifoldMessages.get(i));
         manifolds.add(manifold);
      }
      ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifolds, handTransform, closestPointOnManifold, 1.0, 0.1);
      ReachingManifoldTools.packExtrapolatedTransform(handTransform, closestPointOnManifold, extrapolateRatio, endTransformOnTrajectory);
      reachingManifolds.addAll(reachingManifoldMessages);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeLinearTrajectory(time, trajectoryTime, handTransform, endTransformOnTrajectory);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.SE3};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);

      Graphics3DObject tempGraphic = new Graphics3DObject();
      tempGraphic.transform(closestPointOnManifold);
      tempGraphic.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(tempGraphic);

      Graphics3DObject tempGraphic2 = new Graphics3DObject();
      tempGraphic2.transform(handTransform);
      tempGraphic2.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(tempGraphic2);

      Graphics3DObject tempGraphic3 = new Graphics3DObject();
      tempGraphic3.transform(endTransformOnTrajectory);
      tempGraphic3.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(tempGraphic3);

      PrintTools.info("default final transform");
      System.out.println(closestPointOnManifold);

      return message;
   }
   
   protected void runTest(WholeBodyTrajectoryToolboxMessage message, int maxNumberOfIterations) throws UnreasonableAccelerationException
   {
      List<WaypointBasedTrajectoryMessage> endEffectorTrajectories = message.getEndEffectorTrajectories();
      double t0 = Double.POSITIVE_INFINITY;
      double tf = Double.NEGATIVE_INFINITY;

      if (endEffectorTrajectories != null)
      {
         for (int i = 0; i < endEffectorTrajectories.size(); i++)
         {
            WaypointBasedTrajectoryMessage trajectoryMessage = endEffectorTrajectories.get(i);
            t0 = Math.min(t0, trajectoryMessage.getWaypointTimes().get(0));
            tf = Math.max(t0, trajectoryMessage.getWaypointTimes().get(trajectoryMessage.getWaypoints().size() - 1));

            SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
            // Visualize the position part if it is commanded
            selectionMatrix.resetSelection();
            SelectionMatrix3DMessage angularSelection = trajectoryMessage.getAngularSelectionMatrix();
            SelectionMatrix3DMessage linearSelection = trajectoryMessage.getLinearSelectionMatrix();
            selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
            selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());

            if (!selectionMatrix.isLinearXSelected() && !selectionMatrix.isLinearYSelected() && !selectionMatrix.isLinearZSelected())
               continue; // The position part is not dictated by trajectory, let's not visualize.

            if (visualize)
               scs.addStaticLinkGraphics(createTrajectoryMessageVisualization(trajectoryMessage, 0.01, YoAppearance.AliceBlue()));
         }
      }

      List<ReachingManifoldMessage> reachingManifolds = message.getReachingManifolds();
      if (reachingManifolds != null)
      {
         for (int i = 0; i < reachingManifolds.size(); i++)
         {
            ReachingManifoldMessage reachingManifoldMessage = reachingManifolds.get(i);

            if (visualizeManifold)
               scs.addStaticLinkGraphics(ReachingManifoldTools.createManifoldMessageStaticGraphic(reachingManifoldMessage, 0.01, 10));
         }
      }

      double trajectoryTime = tf - t0;

      commandInputManager.submitMessage(message);

      WholeBodyTrajectoryToolboxOutputStatus solution = runToolboxController(maxNumberOfIterations);

      if (numberOfIterations.getIntegerValue() < maxNumberOfIterations - 1)
         assertNotNull("The toolbox is done but did not report a solution.", solution);
      else
         fail("The toolbox has run for " + maxNumberOfIterations + " without converging nor aborting.");

      if (solution.getPlanningResult() == 4)
      {
         if (visualize)
            visualizeSolution(solution, trajectoryTime / 1000.0);

         trackingTrajectoryWithOutput(message, solution);
      }
      else
      {
         fail("planning result " + solution.getPlanningResult());
      }
   }

   private void trackingTrajectoryWithOutput(WholeBodyTrajectoryToolboxMessage message, WholeBodyTrajectoryToolboxOutputStatus solution)
   {
      List<WaypointBasedTrajectoryMessage> wayPointBasedTrajectoryMessages = message.getEndEffectorTrajectories();
      List<ReachingManifoldMessage> manifoldMessages = message.getReachingManifolds();

      // for every configurations in solution.
      int numberOfConfigurations = solution.getRobotConfigurations().size();
      for (int j = 0; j < numberOfConfigurations; j++)
      {
         // get full robot model.
         KinematicsToolboxOutputStatus configuration = solution.getRobotConfigurations().get(j);
         converter.updateFullRobotModel(configuration);
         FullHumanoidRobotModel outputFullRobotModel = converter.getFullRobotModel();

         double configurationTime = solution.getTrajectoryTimes().get(j);

         // for all way point based trajectory messages.
         for (int i = 0; i < wayPointBasedTrajectoryMessages.size(); i++)
         {
            WaypointBasedTrajectoryMessage trajectory = wayPointBasedTrajectoryMessages.get(i);
            RigidBodyExplorationConfigurationMessage explorationMessage = getRigidBodyExplorationConfigurationMessageHasSameHashCode(message.getExplorationConfigurations(),
                                                                                                                                     trajectory);
            RigidBody rigidBodyOftrajectory = commandConversionHelper.getRigidBody(trajectory.getEndEffectorNameBasedHashCode());

            RigidBody rigidBodyOfOutputFullRobotModel = getRigidBodyHasSameName(outputFullRobotModel, rigidBodyOftrajectory);

            if (rigidBodyOfOutputFullRobotModel == null)
            {
               if (VERBOSE)
                  PrintTools.info("there is no rigid body");
               fail("there is no rigid body");
            }
            else
            {
               RigidBodyTransform solutionRigidBodyTransform = rigidBodyOfOutputFullRobotModel.getBodyFixedFrame().getTransformToWorldFrame();
               Pose3D solutionRigidBodyPose = new Pose3D(solutionRigidBodyTransform);

               if (trajectory.getControlFramePositionInEndEffector() != null)
                  solutionRigidBodyPose.appendTransform(new RigidBodyTransform(new Quaternion(), trajectory.getControlFramePositionInEndEffector()));
               if (trajectory.getControlFrameOrientationInEndEffector() != null)
                  solutionRigidBodyPose.appendTransform(new RigidBodyTransform(trajectory.getControlFrameOrientationInEndEffector(), new Point3D()));

               Pose3D givenRigidBodyPose = HumanoidMessageTools.unpackPose(trajectory, configurationTime);

               double positionError = WholeBodyTrajectoryToolboxHelper.computeTrajectoryPositionError(solutionRigidBodyPose, givenRigidBodyPose,
                                                                                                      explorationMessage, trajectory);

               double orientationError = WholeBodyTrajectoryToolboxHelper.computeTrajectoryOrientationError(solutionRigidBodyPose, givenRigidBodyPose,
                                                                                                            explorationMessage, trajectory);

               if (VERBOSE)
                  PrintTools.info("" + positionError + " " + orientationError);

               if (positionError > TRACKING_TRAJECTORY_POSITION_ERROR_THRESHOLD || orientationError > TRACKING_TRAJECTORY_ORIENTATION_ERROR_THRESHOLD)
               {
                  // TODO consider exploring. or manifold
                  //                  PrintTools.info("rigid body of the solution is far from the given trajectory");
                  //                  fail("rigid body of the solution is far from the given trajectory");
               }
            }
         }

         // TODO
         // for all manifold
         for (int i = 0; i < manifoldMessages.size(); i++)
         {

         }
      }
   }

   private RigidBody getRigidBodyHasSameName(FullHumanoidRobotModel fullRobotModel, RigidBody givenRigidBody)
   {
      RigidBody rootBody = ScrewTools.getRootBody(fullRobotModel.getElevator());
      RigidBody[] allRigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      for (RigidBody rigidBody : allRigidBodies)
         if (givenRigidBody.getName().equals(rigidBody.getName()))
            return rigidBody;
      return null;
   }

   private RigidBodyExplorationConfigurationMessage getRigidBodyExplorationConfigurationMessageHasSameHashCode(List<RigidBodyExplorationConfigurationMessage> rigidBodyExplorationConfigurationMessages,
                                                                                                               WaypointBasedTrajectoryMessage trajectory)
   {
      for (int i = 0; i < rigidBodyExplorationConfigurationMessages.size(); i++)
      {
         RigidBodyExplorationConfigurationMessage message = rigidBodyExplorationConfigurationMessages.get(i);
         if (trajectory.getEndEffectorNameBasedHashCode() == message.getRigidBodyNameBasedHashCode())
            return message;
      }
      return null;
   }

   // TODO
   // Is this for testing ahead put message on planner?
   private SideDependentList<Pose3D> computePrivilegedHandPosesAtPositions(SideDependentList<Point3D> desiredPositions)
   {
      CommandInputManager commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
      FullHumanoidRobotModel desiredFullRobotModel = getRobotModel().createFullRobotModel();

      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));
      HumanoidKinematicsToolboxController whik = new HumanoidKinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel,
                                                                                         new YoGraphicsListRegistry(), new YoVariableRegistry("dummy"));

      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelWithArmsAtMidRange();
      whik.updateRobotConfigurationData(extractRobotConfigurationData(fullRobotModelAtInitialConfiguration));
      whik.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = desiredFullRobotModel.getHand(robotSide);
         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPositions.get(robotSide));
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         commandInputManager.submitMessage(message);
      }

      commandInputManager.submitMessage(KinematicsToolboxMessageFactory.holdRigidBodyCurrentOrientation(desiredFullRobotModel.getChest()));

      int counter = 0;
      int maxIterations = 500;

      while (counter++ <= maxIterations)
      {
         whik.update();
         System.out.println(whik.getSolution().getSolutionQuality());
         snapGhostToFullRobotModel(desiredFullRobotModel);
         scs.tickAndUpdate();
      }

      if (whik.getSolution().getSolutionQuality() > 0.005)
      {
         return null;
      }

      SideDependentList<Pose3D> handPoses = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         Pose3D handPose = new Pose3D();
         handPoses.put(robotSide, handPose);
         RigidBody hand = desiredFullRobotModel.getHand(robotSide);
         RigidBodyTransform transformToWorldFrame = hand.getBodyFixedFrame().getTransformToWorldFrame();
         handPose.set(transformToWorldFrame);
      }
      return handPoses;
   }

   protected static Graphics3DObject createFunctionTrajectoryVisualization(FunctionTrajectory trajectoryToVisualize, double t0, double tf,
                                                                           double timeResolution, double radius, AppearanceDefinition appearance)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      double dT = (tf - t0) / (numberOfWaypoints - 1);

      int radialResolution = 16;
      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfWaypoints, radialResolution, radius);
      Point3DReadOnly[] waypoints = IntStream.range(0, numberOfWaypoints).mapToDouble(i -> t0 + i * dT).mapToObj(trajectoryToVisualize::compute)
                                             .map(pose -> new Point3D(pose.getPosition())).toArray(size -> new Point3D[size]);
      segmentedLine3DMeshGenerator.compute(waypoints);

      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, appearance);
      }
      return graphics;
   }

   protected static Graphics3DObject createTrajectoryMessageVisualization(WaypointBasedTrajectoryMessage trajectoryMessage, double radius,
                                                                          AppearanceDefinition appearance)
   {
      double t0 = trajectoryMessage.getWaypointTimes().get(0);
      double tf = trajectoryMessage.getWaypointTimes().get(trajectoryMessage.getWaypoints().size() - 1);
      double timeResolution = (tf - t0) / trajectoryMessage.getWaypoints().size();
      FunctionTrajectory trajectoryToVisualize = WholeBodyTrajectoryToolboxMessageTools.createFunctionTrajectory(trajectoryMessage);
      return createFunctionTrajectoryVisualization(trajectoryToVisualize, t0, tf, timeResolution, radius, appearance);
   }

   private void visualizeSolution(WholeBodyTrajectoryToolboxOutputStatus solution, double timeResolution) throws UnreasonableAccelerationException
   {
      hideRobot();
      robot.getControllers().clear();

      FullHumanoidRobotModel robotForViz = getRobotModel().createFullRobotModel();
      FloatingInverseDynamicsJoint rootJoint = robotForViz.getRootJoint();
      OneDoFJoint[] joints = FullRobotModelUtils.getAllJointsExcludingHands(robotForViz);

      double trajectoryTime = solution.getTrajectoryTimes().get(solution.getTrajectoryTimes().size() - 1);

      double t = 0.0;

      while (t <= trajectoryTime)
      {
         t += timeResolution;
         KinematicsToolboxOutputStatus frame = findFrameFromTime(solution, t);
         MessageTools.unpackDesiredJointState(frame, rootJoint, joints);

         robotForViz.updateFrames();
         snapGhostToFullRobotModel(robotForViz);
         scs.simulateOneTimeStep();
      }
   }

   private KinematicsToolboxOutputStatus findFrameFromTime(WholeBodyTrajectoryToolboxOutputStatus outputStatus, double time)
   {
      if (time <= 0.0)
         return outputStatus.getRobotConfigurations().get(0);

      else if (time >= outputStatus.getTrajectoryTimes().get(outputStatus.getTrajectoryTimes().size() - 1))
         return outputStatus.getRobotConfigurations().get(outputStatus.getRobotConfigurations().size() - 1);
      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = outputStatus.getTrajectoryTimes().size();

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - outputStatus.getTrajectoryTimes().get(i);
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         KinematicsToolboxOutputStatus frameOne = outputStatus.getRobotConfigurations().get(indexOfFrame - 1);
         KinematicsToolboxOutputStatus frameTwo = outputStatus.getRobotConfigurations().get(indexOfFrame);

         double timeOne = outputStatus.getTrajectoryTimes().get(indexOfFrame - 1);
         double timeTwo = outputStatus.getTrajectoryTimes().get(indexOfFrame);

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         return MessageTools.interpolateMessages(frameOne, frameTwo, alpha);
      }
   }

   protected FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration()
   {
      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, null, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null, null, null);
      drcPerfectSensorReaderFactory.getSensorReader().read();

      return initialFullRobotModel;
   }

   private FullHumanoidRobotModel createFullRobotModelWithArmsAtMidRange()
   {
      FullHumanoidRobotModel robot = createFullRobotModelAtInitialConfiguration();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = robot.getChest();
         RigidBody hand = robot.getHand(robotSide);
         Arrays.stream(ScrewTools.createOneDoFJointPath(chest, hand)).forEach(j -> setJointPositionToMidRange(j));
      }
      return robot;
   }

   private static void setJointPositionToMidRange(OneDoFJoint joint)
   {
      double jointLimitUpper = joint.getJointLimitUpper();
      double jointLimitLower = joint.getJointLimitLower();
      joint.setQ(0.5 * (jointLimitUpper + jointLimitLower));
   }

   private WholeBodyTrajectoryToolboxOutputStatus runToolboxController(int maxNumberOfIterations) throws UnreasonableAccelerationException
   {
      AtomicReference<WholeBodyTrajectoryToolboxOutputStatus> status = new AtomicReference<>(null);
      statusOutputManager.attachStatusMessageListener(WholeBodyTrajectoryToolboxOutputStatus.class, status::set);

      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         for (int i = 0; !toolboxController.isDone() && i < maxNumberOfIterations; i++)
            scs.simulateOneTimeStep();
      }
      else
      {
         for (int i = 0; !toolboxController.isDone() && i < maxNumberOfIterations; i++)
            toolboxUpdater.doControl();
      }
      return status.getAndSet(null);
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getSolverFullRobotModel());

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
               initializationSucceeded.set(toolboxController.initialize());

            if (initializationSucceeded.getBooleanValue())
            {
               try
               {
                  toolboxController.updateInternal();
               }
               catch (InterruptedException | ExecutionException e)
               {
                  e.printStackTrace();
               }
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               numberOfIterations.increment();
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return mainRegistry;
         }

         @Override
         public String getName()
         {
            return mainRegistry.getName();
         }

         @Override
         public String getDescription()
         {
            return null;
         }
      };
   }
}
