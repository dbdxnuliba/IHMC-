package us.ihmc.avatar.largeStepUps;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
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

public abstract class AvatarLargeStepUpsTest implements MultiRobotTestInterface
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

   protected void walkUpToHighStep(ArrayList<Double> stepsHeights) throws SimulationExceededMaximumTimeException
   {

      VariableHeightStairsEnvironment environment = new VariableHeightStairsEnvironment(stepsHeights, 0.6);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      drcSimulationTestHelper.createSimulation("WalkingUpToHighPlatformtest");
      Point3D cameraFix = new Point3D(1.1281, 0.0142, 1.0528);
      Point3D cameraPosition = new Point3D(0.2936, -5.531, 1.7983);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      createTorqueGraph(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FootstepDataListMessage footsteps = createFootstepsForHighStepUp(environment.getStepsCenter());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectory = createPelvisTrajectoryForHighStepUp(environment.getStepsCenter());

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.publishToController(pelvisHeightTrajectory);

      int numberOfSteps = footsteps.getFootstepDataList().size();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * stepTime + 2.0 * initialFinalTransfer + 3.0);

      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());

      assertReachedGoal(footsteps);
      assertTrue(success);
   }

   private FootstepDataListMessage createFootstepsForHighStepUp(ArrayList<Point3D> stepsCenters)
   {
      if (stepsCenters == null || stepsCenters.size() < 2)
      {
         return new FootstepDataListMessage();
      }

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 2 * (stepsCenters.size() - 1));
      FootstepDataListMessage newList = new FootstepDataListMessage();

      for (int i = 0; i < robotSides.length; ++i)
      {
         MovingReferenceFrame soleFrame = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(robotSides[i]);
         FramePose3D footPose = new FramePose3D(soleFrame);
         footPose.changeFrame(ReferenceFrame.getWorldFrame());

         double stepHeight = stepsCenters.get(i / 2 + 1).getZ();
         Point3D location = new Point3D(stepsCenters.get(i / 2 + 1).getX(), footPose.getY(), stepHeight);

         Quaternion quaternion = new Quaternion();
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSides[i], location, quaternion);
         if (i % 2 != 0)
         {
            double initialHeight = stepsCenters.get(i / 2).getZ();
            double deltaZ = stepHeight - initialHeight;

            if (deltaZ > 0)
            {
               footstep.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
               Point3D waypoint1 = footstep.getCustomPositionWaypoints().add();
               Point3D waypoint2 = footstep.getCustomPositionWaypoints().add();
               waypoint1.set(stepsCenters.get(i / 2).getX() - 0.05, footPose.getY(), initialHeight + deltaZ / 2.0);
               waypoint2.set(stepsCenters.get(i / 2).getX(), footPose.getY(), stepHeight + 0.2);
            }
         }

         newList.getFootstepDataList().add().set(footstep);
      }

      return newList;
   }
   
   private PelvisHeightTrajectoryMessage createPelvisTrajectoryForHighStepUp(ArrayList<Point3D> stepsCenters) {

      if (stepsCenters.size() < 2)
      {
         return new PelvisHeightTrajectoryMessage();
      }

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double stepTime = transferTime + swingTime;
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      double nominalPelvisHeight;
      MovingReferenceFrame pelvisZUpFrame = drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame();
      FramePose3D pelvisFrame = new FramePose3D(pelvisZUpFrame);
      pelvisFrame.changeFrame(ReferenceFrame.getWorldFrame());
      nominalPelvisHeight = pelvisFrame.getZ();

      PelvisHeightTrajectoryMessage pelvisHeightTrajectory = new PelvisHeightTrajectoryMessage();
      pelvisHeightTrajectory.setEnableUserPelvisControlDuringWalking(true);
      pelvisHeightTrajectory.setEnableUserPelvisControl(true);

      double time = 1.0 + initialFinalTransfer - transferTime;

      for (int i = 1; i < stepsCenters.size(); ++i)
      {
         double deltaHeight = stepsCenters.get(i).getZ() - stepsCenters.get(i - 1).getZ();
         EuclideanTrajectoryPointMessage waypoint1 = pelvisHeightTrajectory.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
         if (deltaHeight < 0)
         {
            waypoint1.getPosition().setZ(nominalPelvisHeight + 1.1 * deltaHeight);
         }
         else
         {
            waypoint1.getPosition().setZ(nominalPelvisHeight);
         }
         waypoint1.setTime(time + stepTime - 1.0);
         EuclideanTrajectoryPointMessage waypoint2 = pelvisHeightTrajectory.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
         nominalPelvisHeight = pelvisFrame.getZ() + stepsCenters.get(i).getZ();
         waypoint2.getPosition().setZ(nominalPelvisHeight);
         waypoint2.setTime(time + 2 * stepTime - 1.0);
         waypoint2.getLinearVelocity().setZ(0.0);

         time += 2 * stepTime;
      }

      return pelvisHeightTrajectory;
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

}
