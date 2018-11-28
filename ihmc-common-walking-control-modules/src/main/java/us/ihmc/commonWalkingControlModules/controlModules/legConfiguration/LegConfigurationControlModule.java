package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class LegConfigurationControlModule
{
   public enum LegConfigurationType
   {
      STRAIGHTEN, STRAIGHT, COLLAPSE, BENT
   }

   public enum LegControlWeight
   {
      HIGH, MEDIUM, LOW
   }

   private final YoVariableRegistry registry;

   private final PrivilegedJointSpaceCommand privilegedAccelerationCommand = new PrivilegedJointSpaceCommand();

   private final YoEnum<LegConfigurationType> requestedState;

   private final StateMachine<LegConfigurationType, LegControlState> stateMachine;

   private final LegConfigurationController controller;

   private final YoDouble highPrivilegedWeight;
   private final YoDouble mediumPrivilegedWeight;
   private final YoDouble lowPrivilegedWeight;

   private final YoBoolean useFullyExtendedLeg;
   private final YoDouble desiredAngleWhenStraight;
   private final YoDouble desiredAngleWhenExtended;
   private final YoDouble desiredAngleWhenBracing;

   private final YoDouble collapsingDuration;
   private final YoDouble collapsingDurationFractionOfStep;

   private static final int hipPitchJointIndex = 0;
   private static final int kneePitchJointIndex = 1;
   private static final int anklePitchJointIndex = 2;

   private final LegConfigurationControlToolbox toolbox;

   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
                                        LegConfigurationParameters legConfigurationParameters, YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Leg";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      OneDoFJointBasics hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJointBasics kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      OneDoFJointBasics anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);

      toolbox = new LegConfigurationControlToolbox(sidePrefix, kneePitchJoint, legConfigurationParameters, registry);

      privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

      highPrivilegedWeight = new YoDouble(sidePrefix + "HighPrivilegedWeight", registry);
      mediumPrivilegedWeight = new YoDouble(sidePrefix + "MediumPrivilegedWeight", registry);
      lowPrivilegedWeight = new YoDouble(sidePrefix + "LowPrivilegedWeight", registry);

      highPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedHighWeight());
      mediumPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedMediumWeight());
      lowPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedLowWeight());

      useFullyExtendedLeg = new YoBoolean(namePrefix + "UseFullyExtendedLeg", registry);

      desiredAngleWhenStraight = new YoDouble(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenExtended = new YoDouble(namePrefix + "DesiredAngleWhenExtended", registry);
      desiredAngleWhenBracing = new YoDouble(namePrefix + "DesiredAngleWhenBracing", registry);
      desiredAngleWhenStraight.set(legConfigurationParameters.getKneeAngleWhenStraight());
      desiredAngleWhenExtended.set(legConfigurationParameters.getKneeAngleWhenExtended());
      desiredAngleWhenBracing.set(legConfigurationParameters.getKneeAngleWhenBracing());

      collapsingDuration = new YoDouble(namePrefix + "SupportKneeCollapsingDuration", registry);
      collapsingDurationFractionOfStep = new YoDouble(namePrefix + "SupportKneeCollapsingDurationFractionOfStep", registry);
      collapsingDurationFractionOfStep.set(legConfigurationParameters.getSupportKneeCollapsingDurationFractionOfStep());

      // set up states and state machine
      YoDouble time = controllerToolbox.getYoTime();
      requestedState = YoEnum.create(namePrefix + "RequestedState", "", LegConfigurationType.class, registry, true);
      requestedState.set(null);


      stateMachine = setupStateMachine(namePrefix, legConfigurationParameters.attemptToStraightenLegs(), time);
      controller = new LegConfigurationController(sidePrefix, toolbox, controllerToolbox.getFullRobotModel(), registry);

      parentRegistry.addChild(registry);
   }

   private StateMachine<LegConfigurationType, LegControlState> setupStateMachine(String namePrefix, boolean attemptToStraightenLegs, DoubleProvider timeProvider)
   {
      StateMachineFactory<LegConfigurationType, LegControlState> factory = new StateMachineFactory<>(LegConfigurationType.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addStateAndDoneTransition(LegConfigurationType.STRAIGHTEN, new StraighteningKneeControlState(namePrefix, toolbox, registry),
                                        LegConfigurationType.STRAIGHT);
      factory.addState(LegConfigurationType.STRAIGHT, new StraightKneeControlState(toolbox));
      factory.addState(LegConfigurationType.BENT, new BentKneeControlState(toolbox));
      factory.addState(LegConfigurationType.COLLAPSE, new CollapseKneeControlState(namePrefix, toolbox, collapsingDuration, registry));

      for (LegConfigurationType from : LegConfigurationType.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }

      return factory.build(attemptToStraightenLegs ? LegConfigurationType.STRAIGHT : LegConfigurationType.BENT);
   }

   public void initialize()
   {
   }

   public void doControl()
   {
      double desiredStraightLegAngle;
      if (toolbox.useBracingAngle())
         desiredStraightLegAngle = desiredAngleWhenBracing.getDoubleValue();
      else if (useFullyExtendedLeg.getBooleanValue())
         desiredStraightLegAngle = desiredAngleWhenExtended.getDoubleValue();
      else
         desiredStraightLegAngle = desiredAngleWhenStraight.getDoubleValue();
      toolbox.setDesiredStraightLegAngle(desiredStraightLegAngle);

      stateMachine.doActionAndTransition();


      double kneePitchPrivilegedConfigurationWeight;
      LegControlWeight legControlWeight = toolbox.getLegControlWeight().getEnumValue();
      if (legControlWeight == LegControlWeight.LOW)
         kneePitchPrivilegedConfigurationWeight = lowPrivilegedWeight.getDoubleValue();
      else if (legControlWeight == LegControlWeight.MEDIUM)
         kneePitchPrivilegedConfigurationWeight = mediumPrivilegedWeight.getDoubleValue();
      else
         kneePitchPrivilegedConfigurationWeight = highPrivilegedWeight.getDoubleValue();

      LegControlState currentState = stateMachine.getCurrentState();

      controller.setKneePitchPrivilegedConfiguration(currentState.getKneePitchPrivilegedConfiguration());
      controller.setLegConfigurationGains(currentState.getLegConfigurationGains());

      double privilegedKneeAcceleration = controller.computeKneeAcceleration();
      double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
      double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

      privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

      privilegedAccelerationCommand.setWeight(hipPitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(anklePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
   }

   public void setStepDuration(double stepDuration)
   {
      collapsingDuration.set(collapsingDurationFractionOfStep.getDoubleValue() * stepDuration);
   }

   public void setFullyExtendLeg(boolean fullyExtendLeg)
   {
      useFullyExtendedLeg.set(fullyExtendLeg);
   }

   public void prepareForLegBracing()
   {
      toolbox.setUseBracingAngle(true);
   }

   public void doNotBrace()
   {
      toolbox.setUseBracingAngle(false);
   }

   public void setLegControlWeight(LegControlWeight legControlWeight)
   {
      toolbox.setLegControlWeight(legControlWeight);
   }


   public void setKneeAngleState(LegConfigurationType controlType)
   {
      requestedState.set(controlType);
   }

   public LegConfigurationType getCurrentKneeControlState()
   {
      return stateMachine.getCurrentStateKey();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedAccelerationCommand;
   }
}
