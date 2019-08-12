package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.randomizeArmJointPositions;

import java.awt.Color;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private CommandInputManager commandInputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private FullHumanoidRobotModel desiredFullRobotModel;
   private KinematicsStreamingToolboxController toolboxController;

   private HumanoidFloatingRootJointRobot ghost;

   private ScheduledExecutorService scheduledExecutorService;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostApperance);
      ghost = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      ghost.getRootJoint().setPinned(true);
      ghost.setDynamic(false);
      ghost.setGravity(0);
      hideGhost();

      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment()
      {
         @Override
         public List<Robot> getEnvironmentRobots()
         {
            return Collections.singletonList(ghost);
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

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(ghost, fullHumanoidRobotModel);
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
      simulationTestingParameters.setKeepSCSUp(true);
      Random random = new Random(456415);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      KinematicsStreamingToolboxInputMessage inputMessage = new KinematicsStreamingToolboxInputMessage();
      FullHumanoidRobotModel randomizedFullRobotModel = getRobotModel().createFullRobotModel();
      copyFullRobotModelState(drcSimulationTestHelper.getControllerFullRobotModel(), randomizedFullRobotModel);

      for (RobotSide robotSide : RobotSide.values)
      {
         randomizeArmJointPositions(random, robotSide, randomizedFullRobotModel, 0.6);
         RigidBodyBasics hand = randomizedFullRobotModel.getHand(robotSide);
         FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
         desiredPosition.changeFrame(worldFrame);
         KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, desiredPosition);
         message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
         inputMessage.getRigidBodyInputs().add().set(message);
      }

      commandInputManager.submitMessage(inputMessage);
      toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(drcSimulationTestHelper.getControllerFullRobotModel()));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

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
                  snapGhostToFullRobotModel(toolboxController.getDesiredFullRobotModel());
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
