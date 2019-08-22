package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.CenterOfMassTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndCenterOfMassTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final boolean DEBUG = true;

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

   @Test
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564573L);
      //      double epsilon = 1.0e-4;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      double trajectoryTime = 1.0;

      FramePoint3D currentCoMPosition = new FramePoint3D(referenceFrames.getCenterOfMassFrame());
      currentCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());

      FramePoint3D desiredRandomCoMPosition = getRandomCoMPosition(random, referenceFrames.getCenterOfMassFrame());
      Point3D desiredPosition = new Point3D(desiredRandomCoMPosition);

      if (DEBUG)
      {
         System.out.println(desiredPosition);
      }

      desiredRandomCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPosition.set(desiredRandomCoMPosition);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
      }

      CenterOfMassTrajectoryMessage comTrajectoryMessage = new CenterOfMassTrajectoryMessage();

      // - A single point does not work
      // - After the end of the trajectory it goes back to the initial position
      // - The state transition does not happen immediately

      Vector3D zeroLinearVelocity = new Vector3D();
      comTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createEuclideanTrajectoryPointMessage(0.0, currentCoMPosition, zeroLinearVelocity));
      comTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createEuclideanTrajectoryPointMessage(trajectoryTime, desiredPosition, zeroLinearVelocity));

      drcSimulationTestHelper.publishToController(comTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      referenceFrames.updateFrames();
      currentCoMPosition.setToZero(referenceFrames.getCenterOfMassFrame());
      currentCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());
      assertTrue(desiredPosition.epsilonEquals(desiredPosition, 0.01));

      //      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      //
      //      // Hard to figure out how to verify the desired there
      //      //      trajOutput = scs.getVariable("pelvisHeightOffsetSubTrajectoryCubicPolynomialTrajectoryGenerator", "pelvisHeightOffsetSubTrajectoryCurrentValue").getValueAsDouble();
      //      //      assertEquals(desiredPosition.getZ(), trajOutput, epsilon);
      //      // Ending up doing a rough check on the actual height
      //      double pelvisHeight = scs.getVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
      //      assertEquals(desiredPosition.getZ(), pelvisHeight, 0.01);
      //
      //      assertEquals(2, statusMessages.size());
      //      Point3D expectedPosition = new Point3D(Double.NaN, Double.NaN, pelvisHeight);
      //      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisHeightTrajectoryMessage.getSequenceId(),
      //                                                        TrajectoryExecutionStatus.STARTED,
      //                                                        0.0,
      //                                                        "pelvisHeight",
      //                                                        statusMessages.remove(0),
      //                                                        controllerDT);
      //      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisHeightTrajectoryMessage.getSequenceId(),
      //                                                        TrajectoryExecutionStatus.COMPLETED,
      //                                                        trajectoryTime,
      //                                                        expectedPosition,
      //                                                        null,
      //                                                        "pelvisHeight",
      //                                                        statusMessages.remove(0),
      //                                                        1.0e-3,
      //                                                        controllerDT);
      //
      //      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }
   
   protected FramePoint3D getRandomCoMPosition(Random random, MovingReferenceFrame comFrame)
   {
      FramePoint3D desiredRandomCoMPosition = new FramePoint3D(comFrame);
      desiredRandomCoMPosition.set(RandomGeometry.nextPoint3D(random, 0.15, 0.15, 0.10));
      return desiredRandomCoMPosition;
   }
}
