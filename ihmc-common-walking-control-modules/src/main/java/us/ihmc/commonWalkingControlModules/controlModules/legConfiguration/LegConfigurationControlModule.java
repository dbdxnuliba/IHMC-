package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
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

   private static final double minimumDampingScale = 0.2;
   private static final boolean scaleDamping = false;

   private static final double minKneeAngle = 1e-2;

   private final YoVariableRegistry registry;

   private final PrivilegedJointSpaceCommand privilegedAccelerationCommand = new PrivilegedJointSpaceCommand();

   private final YoEnum<LegConfigurationType> requestedState;

   private final StateMachine<LegConfigurationType, LegControlState> stateMachine;

   private final YoDouble highPrivilegedWeight;
   private final YoDouble mediumPrivilegedWeight;
   private final YoDouble lowPrivilegedWeight;


   private final YoDouble kneePitchPrivilegedConfiguration;
   private final YoDouble kneePitchPrivilegedError;

   private final YoDouble jointSpacePAction;
   private final YoDouble jointSpaceDAction;
   private final YoDouble jointSpaceAction;

   private final YoDouble actuatorSpacePAction;
   private final YoDouble actuatorSpaceDAction;
   private final YoDouble actuatorSpaceAction;

   private final YoDouble springSpacePAction;
   private final YoDouble springSpaceDAction;
   private final YoDouble springSpaceAction;

   private final YoDouble privilegedMaxAcceleration;

   private final YoBoolean useFullyExtendedLeg;
   private final YoBoolean useBracingAngle;
   private final YoDouble desiredAngle;
   private final YoDouble desiredAngleWhenStraight;
   private final YoDouble desiredAngleWhenExtended;
   private final YoDouble desiredAngleWhenBracing;

   private final YoDouble collapsingDuration;
   private final YoDouble collapsingDurationFractionOfStep;

   private final YoDouble desiredVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorVelocity;

   private final OneDoFJointBasics kneePitchJoint;

   private final YoDouble dampingActionScaleFactor;

   private static final int hipPitchJointIndex = 0;
   private static final int kneePitchJointIndex = 1;
   private static final int anklePitchJointIndex = 2;

   private double jointSpaceConfigurationGain;
   private double jointSpaceVelocityGain;
   private double actuatorSpaceConfigurationGain;
   private double actuatorSpaceVelocityGain;
   private double springSpaceConfigurationGain;
   private double springSpaceVelocityGain;

   private final LegConfigurationControlToolbox toolbox;

   private final double kneeRangeOfMotion;
   private final double kneeSquareRangeOfMotion;

   private final double thighLength;
   private final double shinLength;

   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
                                        LegConfigurationParameters legConfigurationParameters, YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Leg";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      double kneeLimitUpper = kneePitchJoint.getJointLimitUpper();
      if (Double.isNaN(kneeLimitUpper) || Double.isInfinite(kneeLimitUpper))
         kneeLimitUpper = Math.PI;
      double kneeLimitLower = kneePitchJoint.getJointLimitLower();
      if (Double.isNaN(kneeLimitLower) || Double.isInfinite(kneeLimitLower))
         kneeLimitLower = -Math.PI;
      kneeSquareRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
      kneeRangeOfMotion = kneeLimitUpper - kneeLimitLower;

      toolbox = new LegConfigurationControlToolbox(sidePrefix, kneePitchJoint, legConfigurationParameters, registry);

      OneDoFJointBasics hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJointBasics anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

      highPrivilegedWeight = new YoDouble(sidePrefix + "HighPrivilegedWeight", registry);
      mediumPrivilegedWeight = new YoDouble(sidePrefix + "MediumPrivilegedWeight", registry);
      lowPrivilegedWeight = new YoDouble(sidePrefix + "LowPrivilegedWeight", registry);

      kneePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "KneePitchPrivilegedConfiguration", registry);
      privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);

      kneePitchPrivilegedError = new YoDouble(sidePrefix + "KneePitchPrivilegedError", registry);
      jointSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpacePAction", registry);
      jointSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceDAction", registry);
      jointSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceAction", registry);

      actuatorSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpacePAction", registry);
      actuatorSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpaceDAction", registry);
      actuatorSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpaceAction", registry);

      springSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedSpringSpacePAction", registry);
      springSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedSpringSpaceDAction", registry);
      springSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedSpringSpaceAction", registry);

      highPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedHighWeight());
      mediumPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedMediumWeight());
      lowPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedLowWeight());


      privilegedMaxAcceleration.set(legConfigurationParameters.getPrivilegedMaxAcceleration());

      dampingActionScaleFactor = new YoDouble(namePrefix + "DampingActionScaleFactor", registry);

      useFullyExtendedLeg = new YoBoolean(namePrefix + "UseFullyExtendedLeg", registry);
      useBracingAngle = new YoBoolean(namePrefix + "UseBracingLeg", registry);

      desiredAngle = new YoDouble(namePrefix + "DesiredAngle", registry);

      desiredAngleWhenStraight = new YoDouble(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenExtended = new YoDouble(namePrefix + "DesiredAngleWhenExtended", registry);
      desiredAngleWhenBracing = new YoDouble(namePrefix + "DesiredAngleWhenBracing", registry);
      desiredAngleWhenStraight.set(legConfigurationParameters.getKneeAngleWhenStraight());
      desiredAngleWhenExtended.set(legConfigurationParameters.getKneeAngleWhenExtended());
      desiredAngleWhenBracing.set(legConfigurationParameters.getKneeAngleWhenBracing());

      collapsingDuration = new YoDouble(namePrefix + "SupportKneeCollapsingDuration", registry);
      collapsingDurationFractionOfStep = new YoDouble(namePrefix + "SupportKneeCollapsingDurationFractionOfStep", registry);
      collapsingDurationFractionOfStep.set(legConfigurationParameters.getSupportKneeCollapsingDurationFractionOfStep());

      desiredVirtualActuatorLength = new YoDouble(namePrefix + "DesiredVirtualActuatorLength", registry);
      currentVirtualActuatorLength = new YoDouble(namePrefix + "CurrentVirtualActuatorLength", registry);
      currentVirtualActuatorVelocity = new YoDouble(namePrefix + "CurrentVirtualActuatorVelocity", registry);

      // set up states and state machine
      YoDouble time = controllerToolbox.getYoTime();
      requestedState = YoEnum.create(namePrefix + "RequestedState", "", LegConfigurationType.class, registry, true);
      requestedState.set(null);

      // compute leg segment lengths
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      FramePoint3D hipPoint = new FramePoint3D(hipPitchFrame);
      FramePoint3D kneePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      kneePoint.changeFrame(hipPitchFrame);

      thighLength = hipPoint.distance(kneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      kneePoint.setToZero(kneePitchFrame);
      FramePoint3D anklePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      anklePoint.changeFrame(kneePitchFrame);

      shinLength = kneePoint.distance(anklePoint);

      stateMachine = setupStateMachine(namePrefix, legConfigurationParameters.attemptToStraightenLegs(), time);

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
      if (useBracingAngle.getBooleanValue())
         desiredAngle.set(desiredAngleWhenBracing.getDoubleValue());
      else if (useFullyExtendedLeg.getBooleanValue())
         desiredAngle.set(desiredAngleWhenExtended.getDoubleValue());
      else
         desiredAngle.set(desiredAngleWhenStraight.getDoubleValue());

      stateMachine.doActionAndTransition();

      LegConfigurationGainsReadOnly gains = stateMachine.getCurrentState().getLegConfigurationGains();
      jointSpaceConfigurationGain = gains.getJointSpaceKp();
      jointSpaceVelocityGain = gains.getJointSpaceKd();
      actuatorSpaceConfigurationGain = gains.getActuatorSpaceKp();
      actuatorSpaceVelocityGain = gains.getActuatorSpaceKd();
      springSpaceConfigurationGain = gains.getSpringSpaceKp();
      springSpaceVelocityGain = gains.getSpringSpaceKd();

      double kneePitchPrivilegedConfigurationWeight;
      LegControlWeight legControlWeight = toolbox.getLegControlWeight().getEnumValue();
      if (legControlWeight == LegControlWeight.LOW)
         kneePitchPrivilegedConfigurationWeight = lowPrivilegedWeight.getDoubleValue();
      else if (legControlWeight == LegControlWeight.MEDIUM)
         kneePitchPrivilegedConfigurationWeight = mediumPrivilegedWeight.getDoubleValue();
      else
         kneePitchPrivilegedConfigurationWeight = highPrivilegedWeight.getDoubleValue();

      double privilegedKneeAcceleration = computeKneeAcceleration();
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
      useBracingAngle.set(true);
   }

   public void doNotBrace()
   {
      useBracingAngle.set(false);
   }

   public void setLegControlWeight(LegControlWeight legControlWeight)
   {
      toolbox.setLegControlWeight(legControlWeight);
   }

   private double computeKneeAcceleration()
   {
      double currentPosition = kneePitchJoint.getQ();

      double jointError = kneePitchPrivilegedConfiguration.getDoubleValue() - currentPosition;
      kneePitchPrivilegedError.set(jointError);

      // modify gains based on error. If there's a big error, don't damp velocities
      double percentError = Math.abs(jointError) / (0.5 * kneeRangeOfMotion);
      double dampingActionScaleFactor;
      if (scaleDamping)
         dampingActionScaleFactor = MathTools.clamp(1.0 - (1.0 - minimumDampingScale) * percentError, 0.0, 1.0);
      else
         dampingActionScaleFactor = 1.0;
      this.dampingActionScaleFactor.set(dampingActionScaleFactor);

      double jointSpaceAction = computeJointSpaceAction(dampingActionScaleFactor);
      double actuatorSpaceAction = computeActuatorSpaceAction(dampingActionScaleFactor);
      double springSpaceAction = computeSpringSpaceAction(dampingActionScaleFactor);

      double desiredAcceleration = jointSpaceAction + actuatorSpaceAction + springSpaceAction;

      return MathTools.clamp(desiredAcceleration, privilegedMaxAcceleration.getDoubleValue());
   }

   private double computeJointSpaceAction(double dampingActionScaleFactor)
   {
      double jointError = kneePitchPrivilegedConfiguration.getDoubleValue() - kneePitchJoint.getQ();
      double jointSpaceKp = 2.0 * jointSpaceConfigurationGain / kneeSquareRangeOfMotion;

      double jointSpacePAction = Double.isNaN(jointSpaceKp) ? 0.0 : jointSpaceKp * jointError;
      double jointSpaceDAction = Double.isNaN(jointSpaceVelocityGain) ? 0.0 : dampingActionScaleFactor * jointSpaceVelocityGain * -kneePitchJoint.getQd();

      this.jointSpacePAction.set(jointSpacePAction);
      this.jointSpaceDAction.set(jointSpaceDAction);

      jointSpaceAction.set(jointSpacePAction + jointSpaceDAction);

      return jointSpacePAction + jointSpaceDAction;
   }

   private double computeActuatorSpaceAction(double dampingActionScaleFactor)
   {
      double currentPosition = kneePitchJoint.getQ();

      double desiredVirtualLength = computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue());
      double currentVirtualLength = computeVirtualActuatorLength(currentPosition);

      desiredVirtualActuatorLength.set(desiredVirtualLength);
      currentVirtualActuatorLength.set(currentVirtualLength);

      double currentVirtualVelocity = computeVirtualActuatorVelocity(currentPosition, kneePitchJoint.getQd());
      currentVirtualActuatorVelocity.set(currentVirtualVelocity);

      double virtualError = desiredVirtualLength - currentVirtualLength;

      double actuatorSpacePAction = Double.isNaN(actuatorSpaceConfigurationGain) ? 0.0 : actuatorSpaceConfigurationGain * virtualError;
      double actuatorSpaceDAction = Double.isNaN(actuatorSpaceVelocityGain) ? 0.0 : -dampingActionScaleFactor * actuatorSpaceVelocityGain * currentVirtualVelocity;

      this.actuatorSpacePAction.set(actuatorSpacePAction);
      this.actuatorSpaceDAction.set(actuatorSpaceDAction);

      double actuatorSpaceAcceleration = actuatorSpacePAction + actuatorSpaceDAction;

      double acceleration = computeActuatorAccelerationFromJointAcceleration(currentPosition, kneePitchJoint.getQd(), actuatorSpaceAcceleration);

      this.actuatorSpaceAction.set(acceleration);

      return acceleration;
   }

   private double computeSpringSpaceAction(double dampingActionScaleFactor)
   {
      double currentPosition = kneePitchJoint.getQ();
      double currentVelocity = kneePitchJoint.getQ();

      double desiredVirtualLength = computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue());
      double currentVirtualLength = computeVirtualActuatorLength(currentPosition);

      double currentVirtualVelocity = computeVirtualActuatorVelocity(currentPosition, currentVelocity);

      double virtualError = desiredVirtualLength - currentVirtualLength;

      double springSpacePAction = Double.isNaN(springSpaceConfigurationGain) ? 0.0 : springSpaceConfigurationGain * virtualError;
      double springSpaceDAction = Double.isNaN(springSpaceVelocityGain) ? 0.0 : -dampingActionScaleFactor * springSpaceVelocityGain * currentVirtualVelocity;

      this.springSpacePAction.set(springSpacePAction);
      this.springSpaceDAction.set(springSpaceDAction);

      double springSpaceAcceleration = springSpacePAction + springSpaceDAction;

      double acceleration = computeKneeAccelerationFromLegAcceleration(currentPosition, currentVelocity, springSpaceAcceleration);

      this.springSpaceAction.set(acceleration);

      return acceleration;
   }

   private double computeVirtualActuatorLength(double kneePitchAngle)
   {
      double interiorAngle = Math.PI - Math.max(kneePitchAngle, minKneeAngle);
      return TriangleTools.computeSideLength(thighLength, shinLength, interiorAngle);
   }

   private double computeVirtualActuatorVelocity(double kneePitchAngle, double kneePitchVelocity)
   {
      double virtualLength = computeVirtualActuatorLength(kneePitchAngle);
      return computeVirtualActuatorVelocity(kneePitchAngle, kneePitchVelocity, virtualLength);
   }

   private double computeVirtualActuatorVelocity(double kneePitchAngle, double kneePitchVelocity, double virtualLength)
   {
      double interiorAngle = Math.PI - Math.max(kneePitchAngle, minKneeAngle);
      double interiorVelocity = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchVelocity) : -kneePitchVelocity;
      return TriangleTools.computeSideLengthVelocity(thighLength, shinLength, virtualLength, interiorAngle, interiorVelocity);
   }

   private double computeActuatorAccelerationFromJointAcceleration(double kneePitchAngle, double kneePitchVelocity, double kneePitchAcceleration)
   {
      double actuatorLength = computeVirtualActuatorLength(kneePitchAngle);
      double actuatorVelocity = computeVirtualActuatorVelocity(kneePitchAngle, kneePitchVelocity, actuatorLength);

      double interiorAngle = Math.PI - Math.max(kneePitchAngle, minKneeAngle);
      double interiorVelocity = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchVelocity) : -kneePitchVelocity;
      double interiorAcceleration = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchAcceleration) : -kneePitchAcceleration;

      return TriangleTools.computeSideLengthAcceleration(thighLength, shinLength, actuatorLength, actuatorVelocity, interiorAngle, interiorVelocity, interiorAcceleration);
   }

   private double computeKneeAccelerationFromLegAcceleration(double kneePitchAngle, double kneePitchVelocity, double actuatorAcceleration)
   {
      double actuatorLength = computeVirtualActuatorLength(kneePitchAngle);
      double actuatorVelocity = computeVirtualActuatorVelocity(kneePitchAngle, kneePitchVelocity, actuatorLength);

      double interiorAngle = Math.PI - Math.max(kneePitchAngle, minKneeAngle);
      double interiorVelocity = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchVelocity) : -kneePitchVelocity;

      return TriangleTools.computeInteriorAngleAcceleration(thighLength, shinLength, actuatorLength, actuatorVelocity, actuatorAcceleration, interiorAngle, interiorVelocity);
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
