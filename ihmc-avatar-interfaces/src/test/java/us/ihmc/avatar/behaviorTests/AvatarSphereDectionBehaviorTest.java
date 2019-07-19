package us.ihmc.avatar.behaviorTests;

import org.junit.jupiter.api.*;
import us.ihmc.avatar.*;
import us.ihmc.avatar.testTools.*;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commons.*;
import us.ihmc.commons.thread.*;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.*;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.robotModels.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;
import us.ihmc.simulationconstructionset.util.simulationTesting.*;
import us.ihmc.tools.*;
import us.ihmc.yoVariables.variable.*;

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarSphereDectionBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = new SimulationTestingParameters();
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private final boolean KICK_BALL_BEHAVIOR = false;
   private final boolean SEARCH_AND_KICK_BEHAVIOR = true;

//   private YoBoolean yoDoubeSupport = new YoBoolean();
//   private YoDouble yoTime;
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private StepUpDoor environment = new StepUpDoor(0.5,1.7,0.3);


   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCWalkToLocationBehaviorTest.class + " after class.");
   }

   @BeforeEach
   public void setUp()
   {
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, getRobotModel()); //starts at (0 x,y,z and 0 YawPitchRoll)


   }

   @Test
   public void testWalkToObjectDetected() throws Exception
   {
      if(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         simulationTestingParameters.setKeepSCSUp(true);
      }

      YoBoolean yoDoubleSupport = new YoBoolean("doubleSupport", drcBehaviorTestHelper.getYoVariableRegistry());

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0));


      Ros2Node ros2Node = drcBehaviorTestHelper.getRos2Node();
      FullHumanoidRobotModel fullHumanoidRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();


      drcBehaviorTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);
      //initialize object detection behavior service
      //ObjectDetectorBehaviorService objectDetectorBehaviorService = new ObjectDetectorBehaviorService(getSimpleRobotName(),ros2Node,yoGraphicsListRegistry);
      //WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(getSimpleRobotName(),ros2Node,fullHumanoidRobotModel,humanoidReferenceFrames,walkingControllerParameters);
      //ResetRobotBehavior resetRobotBehavior = new

      if(KICK_BALL_BEHAVIOR)
      {
         KickBallBehavior kickBallBehavior = new KickBallBehavior(getSimpleRobotName(),ros2Node,drcBehaviorTestHelper.getYoTime(),yoDoubleSupport,fullHumanoidRobotModel,humanoidReferenceFrames,getRobotModel());
         kickBallBehavior.initialize();
      }

      else if(SEARCH_AND_KICK_BEHAVIOR)
      {
//         SearchAndKickBehavior searchAndKickBehavior = new SearchAndKickBehavior(getSimpleRobotName(),ros2Node, drcBehaviorTestHelper.getYoTime(),humanoidReferenceFrames, fullHumanoidRobotModel,getRobotModel(), yoDoubleSupport,environment);
         //skpetical about the above line
//         searchAndKickBehavior.initialize();
      }

      //BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

      //drcBehaviorTestHelper.getSimulationConstructionSet().getRootRegistry().addChild(Obj);


   }
}
