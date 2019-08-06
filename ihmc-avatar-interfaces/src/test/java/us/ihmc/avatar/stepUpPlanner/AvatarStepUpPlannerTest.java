package us.ihmc.avatar.stepUpPlanner;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.CollisionAvoidanceManagerMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.StepUpPlannerParametersMessage;
import controller_msgs.msg.dds.StepUpPlannerRequestMessage;
import controller_msgs.msg.dds.StepUpPlannerRespondMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.stepUpPlanner.StepUpPlannerRequester;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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

      ArrayList<Double> heightsVector = new ArrayList<Double>();
      heightsVector.add(stepHeight);

      VariableHeightStairsEnvironment environment = new VariableHeightStairsEnvironment(heightsVector, 0.7);
      //      OffsetAndYawRobotInitialSetup offset = new OffsetAndYawRobotInitialSetup(0.6, 0.0, 0.0, 0.0);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      //      drcSimulationTestHelper.setStartingLocation(offset);
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
      LogTools.info("Sending parameters.");
      boolean ok = requester.publishParametersAndWaitAck(parameters);
      assertTrue(ok);

      StepUpPlannerRequestMessage request = StepUpPlannerRequester.getDefaultFivePhasesRequestMessage(0.6,
                                                                                                      0.0,
                                                                                                      stepHeight,
                                                                                                      desiredLegLength(),
                                                                                                      drcSimulationTestHelper.getReferenceFrames());
      LogTools.info("Sending request.");
      receivedRespond = requester.getRespond(request);
      assertTrue(receivedRespond != null);

      CollisionAvoidanceManagerMessage collisionMessage = createCollisionMessageForHorizontalSurfaces(environment.getPlanarRegionsList());
      drcSimulationTestHelper.publishToController(collisionMessage);

      for (int i = 0; i < receivedRespond.getFoostepMessages().size(); ++i)
      {
         drcSimulationTestHelper.publishToController(receivedRespond.getFoostepMessages().get(i));
      }

      for (int i = 0; i < receivedRespond.getComMessages().size(); ++i)
      {
         drcSimulationTestHelper.publishToController(receivedRespond.getComMessages().get(i));
         drcSimulationTestHelper.publishToController(receivedRespond.getPelvisHeightMessages().get(i));

      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(receivedRespond.getTotalDuration() * 1.2);

      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());

      assertReachedGoal(receivedRespond.getFoostepMessages().getLast());
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
            collisionMessage.setConsiderOnlyEdges(true);
         }
      }

      return collisionMessage;
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