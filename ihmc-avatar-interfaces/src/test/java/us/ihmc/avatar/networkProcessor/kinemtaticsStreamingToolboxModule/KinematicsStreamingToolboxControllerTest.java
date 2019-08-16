package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.randomizeArmJointPositions;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class KinematicsStreamingToolboxControllerTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   private static final YoAppearanceRGBColor operatorApperance = new YoAppearanceRGBColor(Color.BLUE, 0.75);

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private CommandInputManager commandInputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private FullHumanoidRobotModel desiredFullRobotModel;
   private KinematicsStreamingToolboxController toolboxController;

   private HumanoidFloatingRootJointRobot ghost, operator;

   private ScheduledExecutorService scheduledExecutorService;

   /**
    * Returns a <b>new</b> instance of the robot model that will be modified in this test to create
    * ghost robots.
    */
   public abstract DRCRobotModel newRobotModel();

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      ghost = createSCSRobot(newRobotModel(), "ghost", ghostApperance);
      hideRobot(ghost);
      operator = createSCSRobot(newRobotModel(), "operator", operatorApperance);
      hideRobot(operator);

      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment()
      {
         @Override
         public List<Robot> getEnvironmentRobots()
         {
            return Arrays.asList(ghost, operator);
         }
      };
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, testEnvironment);

      desiredFullRobotModel = robotModel.createFullRobotModel();

      mainRegistry = new YoVariableRegistry("main");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsStreamingToolboxCommandConverter(desiredFullRobotModel));
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   yoGraphicsListRegistry,
                                                                   mainRegistry);

      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "test_streaming");
      IHMCRealtimeROS2Publisher<WholeBodyTrajectoryMessage> ros2Publisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                      WholeBodyTrajectoryMessage.class,
                                                                                                      ControllerAPIDefinition.getSubscriberTopicNameGenerator(getSimpleRobotName()));
      toolboxController.setOutputPublisher(ros2Publisher::publish);

      ros2Node.spin();

      scheduledExecutorService = Executors.newScheduledThreadPool(1, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.getSimulationConstructionSet().addYoVariableRegistry(mainRegistry);
   }

   public static HumanoidFloatingRootJointRobot createSCSRobot(DRCRobotModel ghostRobotModel, String robotName, AppearanceDefinition robotAppearance)
   {
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName(robotName);
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), robotAppearance);
      HumanoidFloatingRootJointRobot scsRobot = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      scsRobot.getRootJoint().setPinned(true);
      scsRobot.setDynamic(false);
      scsRobot.setGravity(0);
      return scsRobot;
   }

   private static void hideRobot(HumanoidFloatingRootJointRobot robot)
   {
      robot.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   private static void snapSCSRobotToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel, HumanoidFloatingRootJointRobot robotToSnap)
   {
      JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robotToSnap, fullHumanoidRobotModel);
      jointAnglesWriter.setWriteJointVelocities(false);
      jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      desiredFullRobotModel = null;
      mainRegistry = null;
      yoGraphicsListRegistry = null;
      commandInputManager = null;
      toolboxController = null;

      if (scheduledExecutorService != null)
      {
         scheduledExecutorService.shutdownNow();
         scheduledExecutorService = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testFixedGoal() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(456415);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      // First calibration: Just verifying the proper functioning of the internal state-machine.
      FullHumanoidRobotModel zeroPoseFullRobotModel = getRobotModel().createFullRobotModel();
      zeroPoseFullRobotModel.updateFrames();
      KinematicsStreamingToolboxCalibrationMessage calibrationMessage = new KinematicsStreamingToolboxCalibrationMessage();
      calibrationMessage.getHeadPose().set(zeroPoseFullRobotModel.getHead().getBodyFixedFrame().getTransformToRoot());
      calibrationMessage.getLeftHandPose().set(zeroPoseFullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame().getTransformToRoot());
      calibrationMessage.getRightHandPose().set(zeroPoseFullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToRoot());

      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(controllerFullRobotModel));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(controllerFullRobotModel, getRobotModel(), true, true));
      toolboxController.update();
      assertEquals(KSTState.SLEEP, toolboxController.getCurrentStateKey());
      toolboxController.update();
      assertEquals(KSTState.CALIBRATION, toolboxController.getCurrentStateKey());
      commandInputManager.submitMessage(calibrationMessage);
      toolboxController.update();
      toolboxController.update();
      assertEquals(KSTState.STREAMING, toolboxController.getCurrentStateKey());

      KinematicsStreamingToolboxInputMessage inputMessage = new KinematicsStreamingToolboxInputMessage();
      FullHumanoidRobotModel randomizedFullRobotModel = getRobotModel().createFullRobotModel();
      copyFullRobotModelState(controllerFullRobotModel, randomizedFullRobotModel);

      RigidBodyBasics head = randomizedFullRobotModel.getHead();
      inputMessage.getHeadInput().set(KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(head));

      for (RobotSide robotSide : RobotSide.values)
      {
         randomizeArmJointPositions(random, robotSide, randomizedFullRobotModel, 0.6);
         RigidBodyBasics hand = randomizedFullRobotModel.getHand(robotSide);
         FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
         desiredPosition.changeFrame(worldFrame);
         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPosition);
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         if (robotSide == RobotSide.LEFT)
            inputMessage.getLeftHandInput().set(message);
         else
            inputMessage.getRightHandInput().set(message);
      }

      commandInputManager.submitMessage(inputMessage);

      double simDuration = 5.0;

      YoDouble yoTime = drcSimulationTestHelper.getRobot().getYoTime();
      double terminalTime = yoTime.getValue() + simDuration;

      yoTime.addVariableChangedListener(new VariableChangedListener()
      {
         ScheduledFuture<?> task;

         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (task == null)
               task = scheduledExecutorService.scheduleAtFixedRate(() ->
               {
                  toolboxController.update();
                  snapSCSRobotToFullRobotModel(toolboxController.getDesiredFullRobotModel(), ghost);
               }, 0, 500, TimeUnit.MILLISECONDS);

            if (yoTime.getValue() >= terminalTime)
               task.cancel(true);
         }
      });

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
   }

   @Test
   public void testFixedGoalOperatorOffset() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(456415);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      Vector3D operatorOffset = new Vector3D(-1.73, 1.31, 0.0);

      FullHumanoidRobotModel zeroPoseOperatorFullRobotModel = getRobotModel().createFullRobotModel();
      zeroPoseOperatorFullRobotModel.updateFrames();
      zeroPoseOperatorFullRobotModel.getRootJoint().getJointPose()
                                    .setZ(-zeroPoseOperatorFullRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToRoot().getTranslationZ());
      zeroPoseOperatorFullRobotModel.getRootJoint().getJointPose().getPosition().add(operatorOffset);
      zeroPoseOperatorFullRobotModel.updateFrames();
      snapSCSRobotToFullRobotModel(zeroPoseOperatorFullRobotModel, operator);
      KinematicsStreamingToolboxCalibrationMessage calibrationMessage = new KinematicsStreamingToolboxCalibrationMessage();
      calibrationMessage.getHeadPose().set(zeroPoseOperatorFullRobotModel.getHead().getBodyFixedFrame().getTransformToRoot());
      calibrationMessage.getLeftHandPose().set(zeroPoseOperatorFullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame().getTransformToRoot());
      calibrationMessage.getRightHandPose().set(zeroPoseOperatorFullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToRoot());

      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(controllerFullRobotModel));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(controllerFullRobotModel, getRobotModel(), true, true));
      toolboxController.update();
      assertEquals(KSTState.SLEEP, toolboxController.getCurrentStateKey());
      toolboxController.update();
      assertEquals(KSTState.CALIBRATION, toolboxController.getCurrentStateKey());
      commandInputManager.submitMessage(calibrationMessage);
      toolboxController.update();
      toolboxController.update();
      assertEquals(KSTState.STREAMING, toolboxController.getCurrentStateKey());

      KinematicsStreamingToolboxInputMessage inputMessage = new KinematicsStreamingToolboxInputMessage();
      FullHumanoidRobotModel randomizedOperatorFullRobotModel = getRobotModel().createFullRobotModel();
      copyFullRobotModelState(controllerFullRobotModel, randomizedOperatorFullRobotModel);
      randomizedOperatorFullRobotModel.getRootJoint().getJointPose().getPosition().add(operatorOffset);

      for (RobotSide robotSide : RobotSide.values)
      {
         randomizeArmJointPositions(random, robotSide, randomizedOperatorFullRobotModel, 0.6);
         RigidBodyBasics hand = randomizedOperatorFullRobotModel.getHand(robotSide);
         FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
         desiredPosition.changeFrame(worldFrame);
         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPosition);
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         if (robotSide == RobotSide.LEFT)
            inputMessage.getLeftHandInput().set(message);
         else
            inputMessage.getRightHandInput().set(message);
      }

      RigidBodyBasics head = randomizedOperatorFullRobotModel.getHead();
      HumanoidKinematicsToolboxControllerTest.randomizeKinematicsChainPositions(random, randomizedOperatorFullRobotModel.getChest(), head);
      FrameQuaternion headOrientation = new FrameQuaternion(head.getBodyFixedFrame());
      headOrientation.changeFrame(worldFrame);
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(head, headOrientation);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
      inputMessage.getHeadInput().set(message);

      snapSCSRobotToFullRobotModel(randomizedOperatorFullRobotModel, operator);

      commandInputManager.submitMessage(inputMessage);

      double simDuration = 5.0;

      YoDouble yoTime = drcSimulationTestHelper.getRobot().getYoTime();
      double terminalTime = yoTime.getValue() + simDuration;

      yoTime.addVariableChangedListener(new VariableChangedListener()
      {
         ScheduledFuture<?> task;

         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (task == null)
               task = scheduledExecutorService.scheduleAtFixedRate(() ->
               {
                  toolboxController.update();
                  snapSCSRobotToFullRobotModel(toolboxController.getDesiredFullRobotModel(), ghost);
               }, 0, 500, TimeUnit.MILLISECONDS);

            if (yoTime.getValue() >= terminalTime)
               task.cancel(true);
         }
      });

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
   }

   public static void copyFullRobotModelState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination)
   {
      for (JointStateType stateSelection : JointStateType.values())
         MultiBodySystemTools.copyJointsState(source.getRootJoint().subtreeList(), destination.getRootJoint().subtreeList(), stateSelection);
   }
}
