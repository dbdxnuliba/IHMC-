package us.ihmc.avatar.stepUpPlanner;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.stepUpPlanner.StepUpPlannerRequester;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.VariableHeightStairsEnvironment;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.dataBuffer.DataEntry;

public abstract class AvatarStepUpPlannerTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private final Ros2Node stepUpNode = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_simulation_step_up_node");

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected void walkUpToHighStep(double stepHeight) throws SimulationExceededMaximumTimeException
   {

      boolean useStepUpPlannerTrajectories = true;

      ArrayList<Double> heightsVector = new ArrayList<Double>();
      heightsVector.add(stepHeight);

      VariableHeightStairsEnvironment environment = new VariableHeightStairsEnvironment(heightsVector, 0.7);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableNetworkProcessor(false);
      networkProcessorParameters.enableLocalControllerCommunicator(false); // Force the controller to use FAST_RTPS and not INTRAPROCESS
      drcSimulationTestHelper.setNetworkProcessorParameters(networkProcessorParameters);

      drcSimulationTestHelper.createSimulation("WalkingUpToHighPlatformtest");
      Point3D cameraFix = new Point3D(1.1281, 0.0142, 1.0528);
      Point3D cameraPosition = new Point3D(0.2936, -5.531, 1.7983);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      createTorqueGraph(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));

      StepUpPlannerRespondMessage receivedRespond;
      StepUpPlannerRequester requester = new StepUpPlannerRequester();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      double totalDuration;
      FootstepDataListMessage lastFootStep;

      CollisionAvoidanceManagerMessage collisionMessage = createCollisionMessageForHorizontalSurfaces(environment.getPlanarRegionsList());
      IHMCROS2Publisher<CollisionAvoidanceManagerMessage> collisionPublisher = ROS2Tools.createPublisher(stepUpNode,
                                                                                                         CollisionAvoidanceManagerMessage.class,
                                                                                                         ControllerAPIDefinition.getSubscriberTopicNameGenerator(getRobotModel().getSimpleRobotName()));
      collisionPublisher.publish(collisionMessage);

      if (!useStepUpPlannerTrajectories)
      {
         IHMCROS2Publisher<FootstepDataListMessage> footStepPublisher = ROS2Tools.createPublisher(stepUpNode,
                                                                                                  FootstepDataListMessage.class,
                                                                                                  ControllerAPIDefinition.getSubscriberTopicNameGenerator(getRobotModel().getSimpleRobotName()));
         FootstepDataListMessage footsteps = createFootstepsForHighStepUp(environment.getStepsCenter());
         footStepPublisher.publish(footsteps);

         WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
         int numberOfSteps = footsteps.getFootstepDataList().size();
         double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
         double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();
         totalDuration = numberOfSteps * stepTime + 2.0 * initialFinalTransfer + 3.0;
         lastFootStep = footsteps;
      }
      else
      {

         ReferenceFrame initialCoMFrame = drcSimulationTestHelper.getReferenceFrames().getCenterOfMassFrame();
         FramePose3D comPose = new FramePose3D(initialCoMFrame);
         comPose.changeFrame(ReferenceFrame.getWorldFrame());

         MovingReferenceFrame pelvisZUpFrame = drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame();
         FramePose3D pelvisFrame = new FramePose3D(pelvisZUpFrame);
         pelvisFrame.changeFrame(ReferenceFrame.getWorldFrame());
         double heightDifference = pelvisFrame.getPosition().getZ() - comPose.getPosition().getZ();

         StepUpPlannerParametersMessage parameters = StepUpPlannerRequester.getDefaultFivePhasesParametersMessage(getRobotModel().getWalkingControllerParameters()
                                                                                                                                 .getSteppingParameters(),
                                                                                                                  heightDifference,
                                                                                                                  maxLegLength());

         parameters.setSendComMessages(useStepUpPlannerTrajectories);
         parameters.setIncludeComMessages(false);
         parameters.setComMessagesTopic(ControllerAPIDefinition.getSubscriberTopicNameGenerator(getRobotModel().getSimpleRobotName())
                                                               .generateTopicName(CenterOfMassTrajectoryMessage.class));

         parameters.setSendFootstepMessages(useStepUpPlannerTrajectories);
         parameters.setIncludeFootstepMessages(true);
         parameters.setFootstepMessagesTopic(ControllerAPIDefinition.getSubscriberTopicNameGenerator(getRobotModel().getSimpleRobotName())
                                                                    .generateTopicName(FootstepDataListMessage.class));

         parameters.setSendPelvisHeightMessages(useStepUpPlannerTrajectories);
         parameters.setIncludePelvisHeightMessages(false);
         parameters.setPelvisHeightMessagesTopic(ControllerAPIDefinition.getSubscriberTopicNameGenerator(getRobotModel().getSimpleRobotName())
                                                                        .generateTopicName(PelvisHeightTrajectoryMessage.class));

         LogTools.info("Sending parameters.");
         boolean ok = requester.publishParametersAndWaitAck(parameters);
         assertTrue(ok);

         StepUpPlannerRequestMessage request = StepUpPlannerRequester.getDefaultFivePhasesRequestMessage(new Vector3D(0.6, 0.0, stepHeight),
                                                                                                         desiredLegLength(),
                                                                                                         drcSimulationTestHelper.getReferenceFrames());
         LogTools.info("Sending request.");
         receivedRespond = requester.getRespond(request);
         assertTrue(receivedRespond != null);

         totalDuration = receivedRespond.getTotalDuration() * 1.2;
         lastFootStep = receivedRespond.getFoostepMessages().getLast();
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(totalDuration);

      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());

      assertReachedGoal(lastFootStep);
   }

   private CollisionAvoidanceManagerMessage createCollisionMessageForHorizontalSurfaces(PlanarRegionsList planarRegions)
   {
      CollisionAvoidanceManagerMessage collisionMessage = new CollisionAvoidanceManagerMessage();

      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); ++i)
      {
         Vector3D normal = planarRegions.getPlanarRegion(i).getNormal();
         if (Math.abs(normal.getZ()) >= 0.1)
         {
            PlanarRegionMessage newPlanarRegionMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(planarRegions.getPlanarRegion(i));
            collisionMessage.getPlanarRegionsList().add().set(newPlanarRegionMessage);
         }
      }
      collisionMessage.setConsiderOnlyEdges(true);

      return collisionMessage;
   }

   //Default way of generating footsteps
   private FootstepDataListMessage createFootstepsForHighStepUp(ArrayList<Point3D> stepsCenters)
   {
      if (stepsCenters == null || stepsCenters.size() < 2)
      {
         return new FootstepDataListMessage();
      }

      RobotSide startingFoot = RobotSide.LEFT;

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(startingFoot, 2 * (stepsCenters.size() - 1));
      FootstepDataListMessage newList = new FootstepDataListMessage();

      for (int i = 0; i < robotSides.length; ++i)
      {
         MovingReferenceFrame soleFrame = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(robotSides[i]);
         FramePose3D footPose = new FramePose3D(soleFrame);
         footPose.changeFrame(ReferenceFrame.getWorldFrame());

         double stepHeight = stepsCenters.get(i / 2 + 1).getZ();

         FootstepDataMessage footstep;

         Point3D location = new Point3D(footPose.getX() + 0.6, footPose.getY(), stepHeight);
         Quaternion quaternion = new Quaternion();
         footstep = HumanoidMessageTools.createFootstepDataMessage(robotSides[i], location, quaternion);

         newList.getFootstepDataList().add().set(footstep);
      }

      newList.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());

      return newList;
   }

   private void assertReachedGoal(FootstepDataListMessage footsteps)
   {
      int numberOfSteps = footsteps.getFootstepDataList().size();
      Point3D lastStep = footsteps.getFootstepDataList().get(numberOfSteps - 1).getLocation();
      Point3D nextToLastStep = footsteps.getFootstepDataList().get(numberOfSteps - 2).getLocation();

      Point3D midStance = new Point3D();
      midStance.interpolate(lastStep, nextToLastStep, 0.5);

      Point3D midpoint = new Point3D(midStance);
      midpoint.addZ(1.0);

      Point3D bounds = new Point3D(0.25, 0.25, 0.3);

      BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(midpoint, bounds);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox3D);
   }

   private void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
      }
   }

   private void getPinJoints(Robot robot, List<PinJoint> pinJoints)
   {
      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoints);
      }
   }

   private void createTorqueGraph(SimulationConstructionSet scs, Robot robot)
   {
      StandardSimulationGUI window = scs.getGUI();
      GraphArrayPanel panel = window.getGraphArrayPanel();
      ArrayList<YoGraph> graphs = panel.getGraphsOnThisPanel();

      if (graphs.size() != 0)
         return;

      List<PinJoint> pinJoints = new ArrayList<PinJoint>();

      getPinJoints(robot, pinJoints);

      List<String> torsoJoints = new ArrayList<String>();
      List<String> leftLegJoints = new ArrayList<String>();
      List<String> rightLegJoints = new ArrayList<String>();

      for (PinJoint joint : pinJoints)
      {
         String name = joint.getTauYoVariable().getName();

         if (name.contains("l_leg"))
         {
            leftLegJoints.add(name);
         }
         else if (name.contains("r_leg"))
         {
            rightLegJoints.add(name);
         }
         else if (name.contains("back"))
         {
            torsoJoints.add(name);
         }
      }

      addGraph(scs, torsoJoints);
      addGraph(scs, leftLegJoints);
      addGraph(scs, rightLegJoints);
   }

   private void addGraph(SimulationConstructionSet scs, List<String> torsoJoints)
   {
      scs.setupGraph(torsoJoints.toArray(new String[0]));
   }

   private void printMinMax(SimulationConstructionSet scs)
   {
      System.out.println("-------------------------------------------------------------");
      StandardSimulationGUI window = scs.getGUI();
      GraphArrayPanel panel = window.getGraphArrayPanel();
      ArrayList<YoGraph> graphs = panel.getGraphsOnThisPanel();

      for (YoGraph graph : graphs)
      {
         ArrayList<DataEntry> entries = graph.getEntriesOnThisGraph();
         entries.forEach(entry -> System.out.println(entry.getVariableName() + " Max Torque: [" + Math.max(Math.abs(entry.getMin()), Math.abs(entry.getMax()))
               + "]"));
      }

      System.out.println("-------------------------------------------------------------");
   }

   protected double maxLegLength()
   {
      return 1.15;
   }

   protected double desiredLegLength()
   {
      return 1.05;
   }

}