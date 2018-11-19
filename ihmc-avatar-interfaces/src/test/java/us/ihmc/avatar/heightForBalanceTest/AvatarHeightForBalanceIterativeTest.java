package us.ihmc.avatar.heightForBalanceTest;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StepTilesEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

import java.io.File;
import java.io.PrintWriter;
import java.util.List;

public abstract class AvatarHeightForBalanceIterativeTest extends AvatarHeightForBalanceTestSetup
{
   private boolean succes;
   private double angle;
   private double percentWeight;
   private double delayFractionOfSwing = 0.0;
   private boolean unreasonableFailure;
   private String fileNamePrefix = "angleAndPercentWeight72TilesV4CoP";
   private String fileName = fileNamePrefix;
   protected boolean useNormalRobot = false;
   protected boolean standingPush = false;
   private Integer recursiveIter = 0;
   private Integer numberOfTests = 1;

   public void testIterativePush() throws Exception
   {
      PrintWriter pw = new PrintWriter(new File(fileName + ".csv"));
      StringBuilder sb = new StringBuilder();
      sb.append("Angle");
      sb.append(',');
      sb.append("PercentWeight");
      sb.append('\n');

      int numberOfDirections = 36;
      double increment;
      int incrementCount;
      double startTime =  System.currentTimeMillis();

      for (int i = 0; i < numberOfDirections; i++)
      {
         angle = i * (360 / numberOfDirections) * Math.PI / 180;
         if (Math.cos(angle) > 0.85)
            percentWeight = 0.4;
         else if (Math.cos(angle) < -0.85)
            percentWeight = 0.4;
         else
            percentWeight = 0.2;
         if (standingPush)
         {
            percentWeight = 0.2;
         }
         increment = 0.16;
         incrementCount = 0;
         double percentWeightFail = 0.0;
         for (int k = 0; k < 1000; k++)
         {
            percentWeight += increment;
            if (MathTools.epsilonEquals(percentWeightFail, percentWeight, 0.001) && incrementCount <= 3)
            {
               increment = increment / 2;
               percentWeight = percentWeight - increment;
               incrementCount += 1;
            }
            LogTools.info("Current percentWeight = " + percentWeight + ", current angle = " + angle);

            if (!standingPush)
            {
               testPush();
            }
            else
            {
               testPushStanding();
            }

            if (unreasonableFailure)
            {
               percentWeight = percentWeight - increment;
            }
            else if (!succes && incrementCount > 3)
            {
               percentWeight = percentWeight - increment;
               sb.append(angle);
               sb.append(',');
               sb.append(percentWeight);
               sb.append('\n');
               pw.write(sb.toString());
               double endTime = System.currentTimeMillis();
               double averageTimePerCycle = 3600000* (endTime - startTime) / (i + 1);
               Integer testsRemaining = numberOfTests - (recursiveIter + 1);
               double timeEstimate = (testsRemaining * numberOfDirections + (numberOfDirections - (i + 1))) * averageTimePerCycle;
               LogTools.info("Remaining Time Estimate = " + timeEstimate + " Hour.");
               break;
            }
            else if (!succes)
            {
               percentWeightFail = percentWeight;
               percentWeight = percentWeight - increment;
               increment = increment / 2;
               incrementCount += 1;
            }
         }
      }
      pw.close();
      Assert.assertTrue(true);
      /*
      if (recursiveIter == 0)
      {
         fileName = fileNamePrefix + "Normal";
         useNormalRobot = true;
         recursiveIter += 1;
         testIterativePush();
      }
      else if (recursiveIter == 1)
      {
         fileName = fileNamePrefix + "HalfWay";
         delayFractionOfSwing = 0.4;
         useNormalRobot = false;
         recursiveIter += 1;
         testIterativePush();
      }
      else if (recursiveIter == 2)
      {
         fileName = fileNamePrefix + "HalfWayNormal";
         useNormalRobot = true;
         recursiveIter += 1;
         testIterativePush();
      }
      else if (recursiveIter == 3)
      {
         fileName = fileNamePrefix + "Standing";
         standingPush = true;
         useNormalRobot = false;
         recursiveIter += 1;
         testIterativePush();
      }
      else if (recursiveIter == 4)
      {
         fileName = fileNamePrefix + "StandingNormal";
         useNormalRobot = true;
         recursiveIter += 1;
         testIterativePush();
      }
      */
   }

   @Test
   public void testPush()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);

      CommonAvatarEnvironmentInterface environment = new StepTilesEnvironment(0.25, 0.35, 0.125, 0.5, 6);
      SimulationTestingParameters simulationTestingParameters1 = SimulationTestingParameters.createFromSystemProperties();
      DRCSimulationTestHelper drcSimulationTestHelper1 = new DRCSimulationTestHelper(simulationTestingParameters1, getRobotModel());
      drcSimulationTestHelper1.setTestEnvironment(environment);
      drcSimulationTestHelper1.createSimulation("DRCSimpleFlatGroundScriptTest");

      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      double totalMass1 = fullRobotModel.getTotalMass();

      double z = getForcePointOffsetZInChestFrame();
      PushRobotController pushRobotController1 = new PushRobotController(drcSimulationTestHelper1.getRobot(),
                                                                         fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, z));
      SimulationConstructionSet scs = drcSimulationTestHelper1.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController1.getForceVisualizer());

      try
      {
         drcSimulationTestHelper1.simulateAndBlock(0.5);
         unreasonableFailure = false;
      }
      catch (Exception e)
      {
         LogTools.info(e.getMessage());
         unreasonableFailure = true;
      }

      drcSimulationTestHelper1.publishToController(footsteps);
      SideDependentList<StateTransitionCondition> singleSupportStartConditions1 = new SideDependentList<>();
      SideDependentList<StateTransitionCondition> doubleSupportStartConditions1 = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked") final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs
               .getVariable(sidePrefix + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked") final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs
               .getVariable("WalkingHighLevelHumanoidController", "walkingCurrentState");
         singleSupportStartConditions1.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions1.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      ThreadTools.sleep(1000);

      try
      {
         drcSimulationTestHelper1.simulateAndBlockAndCatchExceptions(3.0);
         unreasonableFailure = false;
      }
      catch (Exception e)
      {
         LogTools.info(e.getMessage());
         unreasonableFailure = true;
      }

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions1.get(RobotSide.LEFT);
      double delay = delayFractionOfSwing * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(Math.cos(angle), Math.sin(angle), 0.0);
      double magnitude = percentWeight * totalMass1 * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController1.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      List<FootstepDataMessage> footstepList = footsteps.getFootstepDataList();
      int size = footstepList.size();
      duration = size * (footsteps.getDefaultSwingDuration() + footsteps.getDefaultTransferDuration());

      try
      {
         succes = drcSimulationTestHelper1.simulateAndBlockAndCatchExceptions(duration + 4.0);
      }
      catch (Exception e)
      {
         LogTools.info(e.getMessage());
         succes = false;
      }

      scs.getYoGraphicsListRegistries().clear();
      scs.cutBuffer();
      scs.cropBuffer();
      scs.getAllVariables().clear();
      scs.getCombinedVarList().removeAllVariables();
      scs.getPlayCycleListeners().clear();
      scs.getDataBuffer().getEntries().clear();

      drcSimulationTestHelper1.destroySimulation();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters1.getShowWindows());

      System.gc();
      Runtime.getRuntime().gc();
   }

   @Test
   public void testPushStanding()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      FootstepDataListMessage footsteps = createStandingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);

      CommonAvatarEnvironmentInterface environment = new StepTilesEnvironment(0.25, 0.35, 0.125, 0.5, 6);
      SimulationTestingParameters simulationTestingParameters1 = SimulationTestingParameters.createFromSystemProperties();
      DRCSimulationTestHelper drcSimulationTestHelper1 = new DRCSimulationTestHelper(simulationTestingParameters1, getRobotModel());
      drcSimulationTestHelper1.setTestEnvironment(environment);
      drcSimulationTestHelper1.createSimulation("DRCSimpleFlatGroundScriptTest");

      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      double totalMass1 = fullRobotModel.getTotalMass();

      double z = getForcePointOffsetZInChestFrame();
      PushRobotController pushRobotController1 = new PushRobotController(drcSimulationTestHelper1.getRobot(),
                                                                         fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, z));
      SimulationConstructionSet scs = drcSimulationTestHelper1.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController1.getForceVisualizer());

      try
      {
         drcSimulationTestHelper1.simulateAndBlock(0.5);
         unreasonableFailure = false;
      }
      catch (Exception e)
      {
         LogTools.info(e.getMessage());
         unreasonableFailure = true;
      }

      drcSimulationTestHelper1.publishToController(footsteps);
      SideDependentList<StateTransitionCondition> singleSupportStartConditions1 = new SideDependentList<>();
      SideDependentList<StateTransitionCondition> doubleSupportStartConditions1 = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked") final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs
               .getVariable(sidePrefix + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked") final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs
               .getVariable("WalkingHighLevelHumanoidController", "walkingCurrentState");
         singleSupportStartConditions1.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions1.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      ThreadTools.sleep(1000);

      try
      {
         drcSimulationTestHelper1.simulateAndBlockAndCatchExceptions(1.5);
         unreasonableFailure = false;
      }
      catch (Exception e)
      {
         LogTools.info(e.getMessage());
         unreasonableFailure = true;
      }

      // push parameters:
      Vector3D forceDirection = new Vector3D(Math.cos(angle), Math.sin(angle), 0.0);
      double magnitude = percentWeight * totalMass1 * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController1.applyForce(forceDirection, magnitude, duration);
      List<FootstepDataMessage> footstepList = footsteps.getFootstepDataList();
      int size = footstepList.size();
      duration = size * (footsteps.getDefaultSwingDuration() + footsteps.getDefaultTransferDuration());

      try
      {
         succes = drcSimulationTestHelper1.simulateAndBlockAndCatchExceptions(duration + 4.0);
      }
      catch (Exception e)
      {
         LogTools.info(e.getMessage());
         succes = false;
      }

      scs.getYoGraphicsListRegistries().clear();
      scs.cutBuffer();
      scs.cropBuffer();
      scs.getAllVariables().clear();
      scs.getCombinedVarList().removeAllVariables();
      scs.getPlayCycleListeners().clear();
      scs.getDataBuffer().getEntries().clear();

      drcSimulationTestHelper1.destroySimulation();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters1.getShowWindows());

      System.gc();
      Runtime.getRuntime().gc();
   }
}
