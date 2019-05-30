package us.ihmc.avatar.straightLegWalking;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import controller_msgs.msg.dds.FootstepDataMessage;
import gnu.trove.list.array.TDoubleArrayList;
import org.apache.poi.ss.formula.functions.T;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootRotationCalculator;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class AvatarStraightLegSingleStepTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private Double stepLength = null;
   private Double stepWidth = null;
   private Double stanceWidth = null;
   private Double stepDownHeight = null;
   private Double stepHeight = null;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      stepLength = null;
      stepWidth = null;
      stanceWidth = null;
      stepDownHeight = null;
      stepHeight = null;
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

      stepLength = null;
      stepWidth = null;
      stanceWidth = null;
      stepDownHeight = null;
      stepHeight = null;
   }

   public void setStepLength(double stepLength)
   {
      this.stepLength = stepLength;
   }

   public void setStepWidth(double stepWidth)
   {
      this.stepWidth = stepWidth;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public void setStepDownHeight(double stepDownHeight)
   {
      this.stepDownHeight = stepDownHeight;
   }

   public void setStepHeight(double stepHeight)
   {
      this.stepHeight = stepHeight;
   }

   @Test
   public void testForwardStep() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage(); //creates a list
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, stepWidth / 2.0, 0.0), new FrameQuaternion()));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void teststep() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      for(int i = 0 ; i < 6; i++)
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                                                new Point3D(stepLength, stepWidth / 2.0, 0.0),
                                                                                                                new FrameQuaternion()));
         if(i<4){stepLength=stepLength +0.6;}
         footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                                                new Point3D(stepLength, -stepWidth / 2.0, 0.0),
                                                                                                              new FrameQuaternion()));
         stepLength=stepLength +0.6;
         //System.out.println(stepLength);
         drcSimulationTestHelper.publishToController(footstepDataListMessage);
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
      }

      //System.out.println(stepLength);
      /*FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                                                new Point3D(stepLength, -stepWidth / 2.0, 0.0),
                                                                                                                  new FrameQuaternion()));*/
      //drcSimulationTestHelper.publishToController(footstepDataListMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      //assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0));

      Point3D center = new Point3D(stepLength,0.0,0.0);
      Vector3D plusMinusvector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusvector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void singleStepUp() throws SimulationExceededMaximumTimeException
   {
      singleStairSetuptest();
      Point3D Step1 = new Point3D(0.6,stepWidth/2.0, 0.0);
      Point3D Step2 = new Point3D(0.6,-stepWidth/2.0, 0.0);
      Point3D Step3 = new Point3D(1.1,stepWidth/2.0, 0.3);
      Point3D Step4 = new Point3D(1.5,-stepWidth/2.0, 0.6);
      Point3D Step5 = new Point3D(1.5,stepWidth/2.0, 0.6);

      ArrayList<Point3D> step = new ArrayList<Point3D>();
      step.add(Step1);
      step.add(Step2);
      step.add(Step3);
      step.add(Step4);
      step.add(Step5);
      RobotSide side = RobotSide.LEFT;
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      for (int i = 0; i < step.size(); i++)
      {
         footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, step.get(i), new FrameQuaternion()));
         side = side.getOppositeSide();
      }

      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0));
      /*
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, Step1, new FrameQuaternion()));
      //drcSimulationTestHelper.publishToController(footsteps);
      //assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, Step2, new FrameQuaternion()));
      //drcSimulationTestHelper.publishToController(footsteps);
      //assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, Step3, new FrameQuaternion()));
      //drcSimulationTestHelper.publishToController(footsteps);
      //assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, Step4, new FrameQuaternion()));
      //drcSimulationTestHelper.publishToController(footsteps);
      //assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, Step5, new FrameQuaternion()));
      //drcSimulationTestHelper.publishToController(footsteps);
      //assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0));
      */


      //Point3D center = new Point3D(,0.0,0.0);
      //Vector3D plusMinusvector = new Vector3D(0.1, 0.1, 0.15);
      //BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusvector);
      //drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }
   @Test
   public void testForwardStepWithPause() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      footstepDataListMessage.getFootstepDataList().clear();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testForwardSteps() throws SimulationExceededMaximumTimeException
   {
      double startingLength = 0.4;
      double nominalLength = 0.75;
      double stepWidth = 0.2;
      int stepsToLength = 3;
      int totalSteps = 20;

      setupTest();

      TDoubleArrayList stepLengths = new TDoubleArrayList();
      stepsToLength++;

      double length = startingLength;
      stepLengths.add(length);
      for (int i = 1; i < Math.max(stepsToLength, totalSteps); i++)
      {
         double alpha = Math.min((double) i / (double) stepsToLength, 1.0);
         length = stepLengths.get(i - 1) + InterpolationTools.linearInterpolate(startingLength, nominalLength, alpha);
         stepLengths.add(length);
      }
      stepLengths.add(length);

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      double width = stepWidth;
      RobotSide side = RobotSide.LEFT;
      for (int i = 0; i < stepLengths.size(); i++)
      {
         FootstepDataMessage message = footstepDataListMessage.getFootstepDataList().add();
         message.set(HumanoidMessageTools.createFootstepDataMessage(side, new Point3D(stepLengths.get(i), width / 2.0, 0.0), new FrameQuaternion()));
         width = -width;
         side = side.getOppositeSide();

//         message.setSwingDuration(0.5);
      }

//      footstepDataListMessage.setAreFootstepsAdjustable(true);

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0));

      Point3D center = new Point3D(stepLengths.get(stepLengths.size() - 1), 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testWideStep() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Point3D step1 = new Point3D(0.0, -stepWidth, 0.0); //right side
      Point3D step2 = new Point3D(0.0, stanceWidth - stepWidth, 0.0); ///left side

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step1, new FrameQuaternion()));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step2, new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      Point3D center = new Point3D();
      center.interpolate(step1, step2, 0.5);
      center.addZ(1.0);
      Vector3D plusMinusVector = new Vector3D(0.1, stanceWidth / 2.0, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testSteppingDown() throws SimulationExceededMaximumTimeException
   {
      runSteppingDown(stepDownHeight, stepDownHeight, stepLength, stanceWidth);
   }

   public void runSteppingDown(double stepDownHeight, double stepHeight, double stepLength, double stanceWidth) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double dropHeight = -stepHeight;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      stepHeights.add(dropHeight);
      stepLengths.add(stepLength);

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, dropHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      setupCamera();

      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();

      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, -stepDownHeight), new Quaternion()));

      drcSimulationTestHelper.publishToController(message);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSteppingDownWithClosing() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double dropHeight = -stepDownHeight;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      stepHeights.add(dropHeight);
      stepLengths.add(stepLength);

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, dropHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      setupCamera();

      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();

      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, dropHeight), new Quaternion()));
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -0.5 * stanceWidth, dropHeight), new Quaternion()));

      drcSimulationTestHelper.publishToController(message);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void singleStairSetuptest() throws SimulationExceededMaximumTimeException
   {
      //CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      //StairsUpAndDownEnvironment stairsUpAndDownEnvironment = new StairsUpAndDownEnvironment();
      AdjustableStairsEnvironment adjustableStairsEnvironment = new AdjustableStairsEnvironment();
      adjustableStairsEnvironment.setStairsParameters(2,1,0.3,0.4);
      adjustableStairsEnvironment.generateTerrains();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), adjustableStairsEnvironment);
      drcSimulationTestHelper.createSimulation("withStairs");

      setupCamera();
      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      setupCamera();
      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
