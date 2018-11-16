package us.ihmc.avatar.heightForBalanceTest;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StepTilesEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StepTilesEnvironmentWithWater;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

import static org.junit.Assert.assertTrue;

public abstract class AvatarHeightForBalanceTestSetup
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected PushRobotController pushRobotController;

   protected double swingTime, transferTime;
   protected double totalMass;

   protected SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   protected SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   protected Double percentWeight;
   protected Double angle;
   protected Boolean succes;

   public abstract double getNominalHeight();

   public abstract double getSlowTransferDuration();

   public abstract double getSlowSwingDuration();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      percentWeight = null;
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         try
         {
            drcSimulationTestHelper.destroySimulation();
         }
         catch (Exception e)
         {
            LogTools.info(e.getMessage());
         }
         drcSimulationTestHelper = null;
      }

      singleSupportStartConditions = null;
      doubleSupportStartConditions = null;
      pushRobotController = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

      percentWeight = null;
   }


   protected abstract DRCRobotModel getRobotModel();

   protected abstract double getSizeScale();

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }


   protected void setupAndRunTest(FootstepDataListMessage message, boolean showTiles, boolean useWaterObject) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      CommonAvatarEnvironmentInterface environment;
      if(useWaterObject)
      {
         environment = new StepTilesEnvironmentWithWater(0.25, 0.35, 0.125, 0.5, 6);
      }
      else if(showTiles)
      {
         environment = new StepTilesEnvironment(0.25, 0.35, 0.125, 0.5, 6);
      }
      else
      {
         environment = new FlatGroundEnvironment();
      }
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      drcSimulationTestHelper.getSimulationConstructionSet().setupVarGroup("HeightForBalanceVars",
                                                                           new String[]{"desiredMomentumRateLinearX","desiredMomentumRateLinearY","desiredMomentumRateLinearZ",
                                                                                 "achievedMomentumRateLinearX","achievedMomentumRateLinearY","achievedMomentumRateLinearZ",
                                                                                 "achievedMomentumRateAngularX","achievedMomentumRateAngularY","achievedMomentumRateAngularZ",
                                                                                 "LinearMomentumX","LinearMomentumY","LinearMomentumZ",
                                                                                 "AngularMomentumX","AngularMomentumY","AngularMomentumZ",
                                                                                 "q_leftHipPitch","qd_leftHipPitch","tau_leftHipPitch",
                                                                                 "q_rightHipPitch","qd_rightHipPitch","tau_rightHipPitch",
                                                                                 "q_leftKneePitch","qd_leftKneePitch","tau_leftKneePitch",
                                                                                 "q_rightKneePitch","qd_rightKneePitch","tau_rightKneePitch",
                                                                                 "q_leftAnklePitch","qd_leftAnklePitch","tau_leftAnklePitch",
                                                                                 "q_rightAnklePitch","qd_rightAnklePitch","tau_rightAnklePitch",
                                                                                 "q_torsoPitch","qd_torsoPitch","tau_torsoPitch",
                                                                                 "torsoErrorRotationVectorX","torsoErrorRotationVectorY","torsoErrorRotationVectorZ",
                                                                                 "pelvisErrorRotationVectorX","pelvisErrorRotationVectorY","pelvisErrorRotationVectorZ",
                                                                                 "perfectCMPX","perfectCMPY","desiredCMPX","desiredCMPY","achievedCMPX","achievedCMPY", "realActualCMPX", "realActualCMPY",
                                                                                 "CenterOfPressureX","CenterOfPressureY","centerOfMassX","centerOfMassY","centerOfMassZ",
                                                                                 "capturePointX","capturePointY","desiredICPX","desiredICPY"});
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();

      double z = getForcePointOffsetZInChestFrame();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(),
                                                    new Vector3D(0, 0, z));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      drcSimulationTestHelper.simulateAndBlock(0.5);

      drcSimulationTestHelper.publishToController(message);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      setupCamera();
      ThreadTools.sleep(1000);
   }

   protected void validateTest(FootstepDataListMessage footsteps) throws SimulationExceededMaximumTimeException
   {
      validateTest(footsteps, true);
   }

   protected void validateTest(FootstepDataListMessage footsteps, boolean simulate) throws SimulationExceededMaximumTimeException
   {
      List<FootstepDataMessage> footstepList = footsteps.getFootstepDataList();
      int size = footstepList.size();

      if (simulate)
      {
         double duration = size * (footsteps.getDefaultSwingDuration() + footsteps.getDefaultTransferDuration());
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 3.0));
      }


   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   protected FootstepDataListMessage createForwardWalkingFootstepMessage()
   {
      double scale = getSizeScale();

      FramePoint3D step1Location = new FramePoint3D(worldFrame, 0.5, -0.125, 0.0);
      FramePoint3D step2Location = new FramePoint3D(worldFrame, 1.0,  0.125, 0.0);
      FramePoint3D step3Location = new FramePoint3D(worldFrame, 1.5, -0.125, 0.0);
      FramePoint3D step4Location = new FramePoint3D(worldFrame, 2.0,  0.125, 0.0);
      FramePoint3D step5Location = new FramePoint3D(worldFrame, 2.5, -0.125, 0.0);
      FramePoint3D step6Location = new FramePoint3D(worldFrame, 3.0,  0.125, 0.0);

      step1Location.scale(scale);
      step2Location.scale(scale);
      step3Location.scale(scale);
      step4Location.scale(scale);
      step5Location.scale(scale);
      step6Location.scale(scale);

      FootstepDataMessage message1 = createFootstepDataMessage(RobotSide.RIGHT, step1Location);
      FootstepDataMessage message2 = createFootstepDataMessage(RobotSide.LEFT, step2Location);
      FootstepDataMessage message3 = createFootstepDataMessage(RobotSide.RIGHT, step3Location);
      FootstepDataMessage message4 = createFootstepDataMessage(RobotSide.LEFT, step4Location);
      FootstepDataMessage message5 = createFootstepDataMessage(RobotSide.RIGHT, step5Location);
      FootstepDataMessage message6 = createFootstepDataMessage(RobotSide.LEFT, step6Location);

      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.getFootstepDataList().add().set(message1);
      message.getFootstepDataList().add().set(message2);
      message.getFootstepDataList().add().set(message3);
      message.getFootstepDataList().add().set(message4);
      message.getFootstepDataList().add().set(message5);
      message.getFootstepDataList().add().set(message6);

      message.setAreFootstepsAdjustable(true);

      return message;
   }

   protected FootstepDataListMessage createStandingFootstepMessage()
   {
      double scale = getSizeScale();

      FramePoint3D step1Location = new FramePoint3D(worldFrame, 0.0, -0.125, 0.0);

      step1Location.scale(scale);

      FootstepDataMessage message1 = createFootstepDataMessage(RobotSide.RIGHT, step1Location);

      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      //message.getFootstepDataList().add().set(message1);

      message.setAreFootstepsAdjustable(true);

      return message;
   }

   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep)
   {
      return createFootstepDataMessage(robotSide, placeToStep, new FrameQuaternion());
   }

   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep, FrameQuaternion orientation)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint3D placeToStepInWorld = new FramePoint3D(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);
      orientation.changeFrame(worldFrame);

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(orientation);
      footstepData.setRobotSide(robotSide.toByte());

      return footstepData;
   }

   protected class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
   }

   protected class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean testCondition(double time)
      {
         if (side == RobotSide.LEFT)
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
         }
         else
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
         }
      }
   }
}