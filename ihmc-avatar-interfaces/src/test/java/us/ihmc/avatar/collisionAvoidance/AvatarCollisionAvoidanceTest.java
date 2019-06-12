package us.ihmc.avatar.collisionAvoidance;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.CollisionManagerMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.VariableHeightStairsEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class AvatarCollisionAvoidanceTest implements MultiRobotTestInterface
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

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FootstepDataListMessage footsteps = createFootstepsForHighStepUp(environment.getStepsCenter());

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      CollisionManagerMessage collisionMessage = createCollisionMessage(environment.getPlanarRegionsList());

      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.publishToController(collisionMessage);

      int numberOfSteps = footsteps.getFootstepDataList().size();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * stepTime + 2.0 * initialFinalTransfer + 3.0);

      assertReachedGoal(footsteps);
      assertTrue(success);
   }

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

         Point3D location = new Point3D(stepsCenters.get(i / 2 + 1).getX(), footPose.getY(), stepHeight);
         Quaternion quaternion = new Quaternion();
         footstep = HumanoidMessageTools.createFootstepDataMessage(robotSides[i], location, quaternion);

         newList.getFootstepDataList().add().set(footstep);
      }

      newList.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());

      return newList;
   }

   private CollisionManagerMessage createCollisionMessage(PlanarRegionsList planarRegions)
   {
      CollisionManagerMessage collisionMessage = new CollisionManagerMessage();
      collisionMessage.setTest((float) 0.38);

      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); ++i)
      {
         Vector3D normal = planarRegions.getPlanarRegion(i).getNormal();
         if (Math.abs(normal.getZ()) < 0.1)
         {
            PlanarRegionMessage newPlanarRegionMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(planarRegions.getPlanarRegion(i));
            collisionMessage.getPlanarRegionsList().add().set(newPlanarRegionMessage);
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

}
