package us.ihmc.avatar.stepUptest;

import static us.ihmc.robotics.Assert.*;

import controller_msgs.msg.dds.*;
import io.netty.util.concurrent.*;
import javafx.geometry.*;
import org.apache.commons.lang3.ObjectUtils.*;
import org.junit.jupiter.api.*;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commons.thread.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.footstep.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;
import us.ihmc.simulationconstructionset.util.simulationTesting.*;
import us.ihmc.tools.*;

import java.awt.*;

public abstract class AvatarStepUp implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   //private final AtlasR

   @BeforeAll
   public  void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterAll

   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if(drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + "after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   //start writing your step up code here now
   @Test
   public void stepUp() throws SimulationExceededMaximumTimeException
   {
      setTestEnvironment();

      FootstepDataListMessage footsteps = createFootSteps();

      drcSimulationTestHelper.publishToController(footsteps);

      WalkingControllerParameters walkingControllerParameters = new WalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTouchdownTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime +2.0*initialFinalTransfer + 3.0);

      assertreached(footsteps);
   }

   private FootstepDataListMessage createFootSteps()
   {
      //create a list of desired footstep
      double stepWidth = 0.11;
      double[][] steps = {{0.3,stepWidth,0.0},{0.3,-stepWidth,0.0},{0.6,stepWidth,0.4},{0.8,-stepWidth,0.8},{0.8,stepWidth,0.8}};
      //Point3D[][] steps2 =  {{new Point3D(0.0, 0.0, 0.0)},{0.3,-stepWidth,0.0},{0.6,stepWidth,0.4},{0.8,-stepWidth,0.8},{0.8,stepWidth,0.8}};


      ///create a object for the footstepdtatalistmessage class
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      //create a side variable and a quaternion variable to create a data message

      RobotSide side = RobotSide.LEFT;


      //add them to a object of class footstepdatalistmessage
      for (int i = 0 ; i < steps.length; i++)
      {
         //store the desired step location as Point3D object
         Point3D point3D = new Point3D(steps[i]);
         FrameQuaternion frameQuaternion = new FrameQuaternion();
         footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, point3D, frameQuaternion));
         side.getOppositeSide();
      }

      return footstepDataListMessage;
   }

   private void assertreached(FootstepDataListMessage footsteps)
   {
      int numberofsteps = footsteps.getFootstepDataList().size();
      Point3D lastStep = footsteps.getFootstepDataList().get(numberofsteps-1).getLocation();
      Point3D nextToLastStep = footsteps.getFootstepDataList().get(numberofsteps - 2).getLocation();

      Point3D midStance = new Point3D();
      midStance.interpolate(lastStep, nextToLastStep, 0.5);

      Point3D midpoint = new Point3D(midStance);
      midpoint.addZ(1.0);

      Point3D bounds = new Point3D(0.25, 0.25, 0.3);

      BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(midpoint, bounds);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox3D);
   }
   private void setTestEnvironment() throws SimulationExceededMaximumTimeException
   {
      //create environment
      AdjustableStairsEnvironment adjustableStairsEnvironment = new AdjustableStairsEnvironment();
      adjustableStairsEnvironment.setCourseStartDistance(0.5);
      adjustableStairsEnvironment.setLandingPlatformParameters(0,0,0,0);
      adjustableStairsEnvironment.setRailingParameters(0,0,0,0,0,false);
      adjustableStairsEnvironment.setStairsParameters(2,1.0,0.4,0.5);
      adjustableStairsEnvironment.generateTerrains();

      //pass this reference to the simulator
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), adjustableStairsEnvironment);
      //drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, DRCRobotModel atlasRobotModel, adjustableStairsEnvironment);
      drcSimulationTestHelper.createSimulation("TwoStairEnvironment");

      setUpCamera();
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20);

      assertTrue(success);

   }

   private void setUpCamera()
   {
      Point3D cameraFix = new Point3D(0,0,0);
      Point3D cameraPosition = new Point3D(0,0,0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}

