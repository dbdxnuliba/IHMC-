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
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
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
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.SingleStepEnvironment;
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

   protected void walkUpToHighStep(double stepHeight) throws SimulationExceededMaximumTimeException
   {

      SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 1.0);
      OffsetAndYawRobotInitialSetup offset = new OffsetAndYawRobotInitialSetup(0.7, 0.0, 0.0, 0.0);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      drcSimulationTestHelper.setStartingLocation(offset);
      drcSimulationTestHelper.createSimulation("WalkingUpToHighPlatformtest");
      Point3D cameraFix = new Point3D(1.1281, 0.0142, 1.0528);
      Point3D cameraPosition = new Point3D(0.2936, -5.531, 1.7983);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      createTorqueGraph(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FootstepDataListMessage footsteps = createFootstepsForHighStepUp(stepHeight);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      double nominalPelvisHeight;
      MovingReferenceFrame pelvisZUpFrame = drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame();
      FramePose3D pelvisFrame = new FramePose3D(pelvisZUpFrame);
      pelvisFrame.changeFrame(ReferenceFrame.getWorldFrame());
      nominalPelvisHeight = pelvisFrame.getZ();

      PelvisHeightTrajectoryMessage pelvisHeightTrajectory = new PelvisHeightTrajectoryMessage();
      pelvisHeightTrajectory.setEnableUserPelvisControlDuringWalking(true);
      pelvisHeightTrajectory.setEnableUserPelvisControl(true); // ?
      EuclideanTrajectoryPointMessage waypoint1 = pelvisHeightTrajectory.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint1.getPosition().setZ(1.05 * nominalPelvisHeight);
      waypoint1.setTime(initialFinalTransfer + walkingControllerParameters.getDefaultTransferTime() + 0.5);
      EuclideanTrajectoryPointMessage waypoint2 = pelvisHeightTrajectory.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint2.getPosition().setZ(nominalPelvisHeight + stepHeight);
      waypoint2.setTime(initialFinalTransfer + stepTime + 0.5);
      waypoint1.getLinearVelocity().setZ(0.0);

      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.publishToController(pelvisHeightTrajectory);

      int numberOfSteps = footsteps.getFootstepDataList().size();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * stepTime + 2.0 * initialFinalTransfer + 3.0);

      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());

      assertReachedGoal(footsteps);
      assertTrue(success);
   }

   protected void walkDownFromHighStep(double stepHeight) throws SimulationExceededMaximumTimeException
   {

      SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 1.0);
      OffsetAndYawRobotInitialSetup offset = new OffsetAndYawRobotInitialSetup(1.8, 0.0, stepHeight, 0.0);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      drcSimulationTestHelper.setStartingLocation(offset);
      drcSimulationTestHelper.createSimulation("WalkingDownFromHighPlatformtest");
      Point3D cameraFix = new Point3D(1.1281, 0.0142, 1.0528);
      Point3D cameraPosition = new Point3D(0.2936, -5.531, 1.7983);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      createTorqueGraph(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FootstepDataListMessage footsteps = createFootstepsForHighStepDown(stepHeight);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      double nominalPelvisHeight;
      MovingReferenceFrame pelvisZUpFrame = drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame();
      FramePose3D pelvisFrame = new FramePose3D(pelvisZUpFrame);
      pelvisFrame.changeFrame(ReferenceFrame.getWorldFrame());
      nominalPelvisHeight = pelvisFrame.getZ();
      PelvisHeightTrajectoryMessage pelvisHeightTrajectory = new PelvisHeightTrajectoryMessage();
      pelvisHeightTrajectory.setEnableUserPelvisControlDuringWalking(true);
      pelvisHeightTrajectory.setEnableUserPelvisControl(true); // ?
      EuclideanTrajectoryPointMessage waypoint1 = pelvisHeightTrajectory.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint1.getPosition().setZ(nominalPelvisHeight - 0.9 * stepHeight);
      waypoint1.setTime(initialFinalTransfer + walkingControllerParameters.getDefaultTransferTime());
      EuclideanTrajectoryPointMessage waypoint2 = pelvisHeightTrajectory.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint2.getPosition().setZ(nominalPelvisHeight - stepHeight);
      waypoint2.setTime(initialFinalTransfer + stepTime);
      waypoint1.getLinearVelocity().setZ(0.0);

      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.publishToController(pelvisHeightTrajectory);

      int numberOfSteps = footsteps.getFootstepDataList().size();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * stepTime + 2.0 * initialFinalTransfer + 3.0);

      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());

      assertReachedGoal(footsteps);
      assertTrue(success);
   }

   private FootstepDataListMessage createFootstepsForHighStepUp(double stepHeight)
   {
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 2);
      FootstepDataListMessage newList = new FootstepDataListMessage();

      for (int i = 0; i < robotSides.length; ++i)
      {
         MovingReferenceFrame soleFrame = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(robotSides[i]);
         FramePose3D footPose = new FramePose3D(soleFrame);
         footPose.changeFrame(ReferenceFrame.getWorldFrame());

         Point3D location = new Point3D(1.2, footPose.getY(), stepHeight);
         Quaternion quaternion = new Quaternion();
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSides[i], location, quaternion);
         if (i == 1)
         {
            footstep.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
            Point3D waypoint1 = footstep.getCustomPositionWaypoints().add();
            Point3D waypoint2 = footstep.getCustomPositionWaypoints().add();
            waypoint1.set(footPose.getX() - 0.1, footPose.getY(), stepHeight / 2.0);
            waypoint2.set(footPose.getX(), footPose.getY(), stepHeight + 0.2);
         }

         newList.getFootstepDataList().add().set(footstep);
      }

      return newList;
   }

   private FootstepDataListMessage createFootstepsForHighStepDown(double stepHeight)
   {
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 2);
      FootstepDataListMessage newList = new FootstepDataListMessage();

      for (int i = 0; i < robotSides.length; ++i)
      {
         MovingReferenceFrame soleFrame = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(robotSides[i]);
         FramePose3D footPose = new FramePose3D(soleFrame);
         footPose.changeFrame(ReferenceFrame.getWorldFrame());

         Point3D location = new Point3D(2.2, footPose.getY(), 0.0);
         Quaternion quaternion = new Quaternion();
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSides[i], location, quaternion);

         newList.getFootstepDataList().add().set(footstep);
      }

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

}
