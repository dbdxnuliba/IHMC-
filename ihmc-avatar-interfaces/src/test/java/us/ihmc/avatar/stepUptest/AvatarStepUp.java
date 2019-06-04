package us.ihmc.avatar.stepUptest;

import static us.ihmc.robotics.Assert.*;

import controller_msgs.msg.dds.*;
import org.jfree.chart.plot.dial.DialPointer.*;
import org.junit.jupiter.api.*;
import us.ihmc.avatar.*;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commons.thread.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.*;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;
import us.ihmc.simulationconstructionset.util.simulationTesting.*;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

import us.ihmc.tools.*;
import us.ihmc.yoVariables.dataBuffer.*;

import java.util.*;

public abstract class AvatarStepUp implements MultiRobotTestInterface
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   //private final AtlasR

   @BeforeEach
   public  void showMemoryUsageBeforeTest()
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
   public void stepUpSmall() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.3;

      walkingStair(stepHeight);

   }

   @Test
   public void stepUpBig() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.5;
      //OffsetAndYawRobotInitialSetup
      //drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.5, 0.0, 0.0, 0.0));
      walkingStair(stepHeight);
   }

   private void walkingStair(double stepHeight) throws SimulationExceededMaximumTimeException
   {
      setTestEnvironment(stepHeight);

      FootstepDataListMessage footsteps = createFootSteps(stepHeight);

      drcSimulationTestHelper.publishToController(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTouchdownTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime +2.0*initialFinalTransfer + 3.0);

      assertreached(footsteps);
   }

   private FootstepDataListMessage createFootSteps(double stepHeight)
   {
      //create a list of desired footstep
      double stepWidth = 0.11;
      double[][] steps = {{0.6, stepWidth, 0.0}, {0.6, -stepWidth, 0.0}, {0.8, stepWidth, 0.0}, {1.2, -stepWidth, stepHeight}, {1.2, stepWidth, stepHeight}};//,{0.6,stepWidth,0.4},{0.8,-stepWidth,0.8},{0.8,stepWidth,0.8}};

      /*else
      {
         double[][] steps = {{0.3, stepWidth, 0.0}, {0.6, -stepWidth, 0.0}, {0.75, stepWidth, 0.0}, {1.1, -stepWidth, stepHeight}, {1.1, stepWidth, stepHeight}};
         //Point3D[][] steps2 =  {{new Point3D(0.0, 0.0, 0.0)},{0.3,-stepWidth,0.0},{0.6,stepWidth,0.4},{0.8,-stepWidth,0.8},{0.8,stepWidth,0.8}};
      }*/

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
         side = side.getOppositeSide();
      }

      return footstepDataListMessage;
   }

   private static void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);

      for(Joint child : joint. getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
      }
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
   private void setTestEnvironment(double stepHeight) throws SimulationExceededMaximumTimeException
   {
    /*      //create environment
      AdjustableStairsEnvironment adjustableStairsEnvironment = new AdjustableStairsEnvironment();
      //adjustableStairsEnvironment.setCourseStartDistance(0.5);
      //adjustableStairsEnvironment.setLandingPlatformParameters(0,0,0,0);
      //adjustableStairsEnvironment.setRailingParameters(0,0,0,0,0,false);
      adjustableStairsEnvironment.setStairsParameters(2,1.0,0.4,0.5);
      adjustableStairsEnvironment.generateTerrains();
*/



      SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 0.5);

      //pass this reference to the simulator
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      //drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, DRCRobotModel atlasRobotModel, adjustableStairsEnvironment);
      drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.5, 0.0, 0.0, 0.0));
      drcSimulationTestHelper.createSimulation("TwoStairEnvironment");

      setUpCamera();
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20);
      createTorqueGraphs(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));
      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());
      assertTrue(success);

   }

   private void setUpCamera()
   {
      Point3D cameraFix = new Point3D(0.0,0.0,0.89);
      Point3D cameraPosition = new Point3D(10.0,2.0,1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void getPinJoints(Robot robot, List<PinJoint> pinJoint)
   {
      for(Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoint);
      }
   }
   private void createTorqueGraphs(SimulationConstructionSet scs, Robot robot)
   {

      List<PinJoint> pinJoints = new ArrayList<>();

      getPinJoints(robot, pinJoints);

      List<String> torsojoints = new ArrayList<String>();
      List<String> leftlegjoints = new ArrayList<String>();
      List<String> rightlegjoints = new ArrayList<String>();

      for (PinJoint joint : pinJoints)
      {
         String name = joint.getTauYoVariable().getName();

         if (name.contains("l_leg"))
         {
            leftlegjoints.add(name);
         }
         else if (name.contains("r_leg"))
         {
            rightlegjoints.add(name);
         }
         else if (name.contains("back"))
         {
            torsojoints.add(name);
         }

         addGraph(scs, torsojoints);
         addGraph(scs, leftlegjoints);
         addGraph(scs, rightlegjoints);
      }
   }

      private void addGraph(SimulationConstructionSet scs, List<String> joint)
      {
         scs.setupGraph(joint.toArray(new String[0])); //casting it to string object so that it can be passed to plot graph
      }

      private void printMinMax(SimulationConstructionSet scs)
      {
         StandardSimulationGUI window = scs.getGUI();
         GraphArrayPanel panel = window.getGraphArrayPanel();
         ArrayList<YoGraph> graphs = panel.getGraphsOnThisPanel();

         if (graphs.size() != 0)
         {
            return;
         }

         for(YoGraph graph: graphs)
         {
            ArrayList<DataEntry> entries = graph. getEntriesOnThisGraph();
            entries.forEach(entry -> System.out.println(entry.getVariableName() + "Max Torque: [" + Math.max(Math.abs(entry.getMin()), Math.abs(entry.getMax()))+ "]"));
         }
      }

}

