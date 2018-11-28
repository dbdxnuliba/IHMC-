package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LegConfigurationController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final double minKneeAngle = 1e-2;

   private static final double minimumDampingScale = 0.2;
   private static final boolean scaleDamping = false;

   private final LegConfigurationControlToolbox toolbox;
   private final OneDoFJointBasics kneePitchJoint;

   private final YoDouble privilegedMaxAcceleration;
   private final YoDouble dampingActionScaleFactor;

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

   private final YoDouble desiredVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorVelocity;

   private final YoDouble desiredVirtualSpringLength;
   private final YoDouble currentVirtualSpringLength;
   private final YoDouble currentVirtualSpringVelocity;

   private final double thighLength;
   private final double shinLength;

   private LegConfigurationGainsReadOnly legConfigurationGains;

   public LegConfigurationController(String sidePrefix, LegConfigurationControlToolbox toolbox, FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      this.kneePitchJoint = toolbox.getKneePitchJoint();

      String namePrefix = sidePrefix + "Leg";

      kneePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "KneePitchPrivilegedConfiguration", registry);
      kneePitchPrivilegedError = new YoDouble(sidePrefix + "KneePitchPrivilegedError", registry);

      privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);
      dampingActionScaleFactor = new YoDouble(namePrefix + "DampingActionScaleFactor", registry);

      jointSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpacePAction", registry);
      jointSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceDAction", registry);
      jointSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceAction", registry);

      actuatorSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpacePAction", registry);
      actuatorSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpaceDAction", registry);
      actuatorSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpaceAction", registry);

      springSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedSpringSpacePAction", registry);
      springSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedSpringSpaceDAction", registry);
      springSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedSpringSpaceAction", registry);

      desiredVirtualActuatorLength = new YoDouble(namePrefix + "DesiredVirtualActuatorLength", registry);
      currentVirtualActuatorLength = new YoDouble(namePrefix + "CurrentVirtualActuatorLength", registry);
      currentVirtualActuatorVelocity = new YoDouble(namePrefix + "CurrentVirtualActuatorVelocity", registry);

      desiredVirtualSpringLength = new YoDouble(namePrefix + "DesiredVirtualSpringLength", registry);
      currentVirtualSpringLength = new YoDouble(namePrefix + "CurrentVirtualSpringLength", registry);
      currentVirtualSpringVelocity = new YoDouble(namePrefix + "CurrentVirtualSpringVelocity", registry);

      privilegedMaxAcceleration.set(toolbox.getParameters().getPrivilegedMaxAcceleration());

      // compute leg segment lengths
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

      parentRegistry.addChild(registry);
   }

   public void setKneePitchPrivilegedConfiguration(double kneePitchPrivilegedConfiguration)
   {
      this.kneePitchPrivilegedConfiguration.set(kneePitchPrivilegedConfiguration);
   }

   public void setLegConfigurationGains(LegConfigurationGainsReadOnly legConfigurationGains)
   {
      this.legConfigurationGains = legConfigurationGains;
   }



   private double computeKneeAcceleration()
   {
      double currentPosition = kneePitchJoint.getQ();

      double jointError = kneePitchPrivilegedConfiguration.getDoubleValue() - currentPosition;
      kneePitchPrivilegedError.set(jointError);

      // modify gains based on error. If there's a big error, don't damp velocities
      double percentError = Math.abs(jointError) / (0.5 * toolbox.getKneeRangeOfMotion());
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
      double jointSpaceKp = legConfigurationGains.hasJointSpaceKp() ? 0.0 : 2.0 * legConfigurationGains.getJointSpaceKp() / toolbox.getKneeSquareRangeOfMotion();
      double jointSpaceKd = legConfigurationGains.hasJointSpaceKd() ? 0.0 : dampingActionScaleFactor * legConfigurationGains.getJointSpaceKd();

      jointSpacePAction.set(jointSpaceKp * jointError);
      jointSpaceDAction.set(jointSpaceKd * -kneePitchJoint.getQd());
      jointSpaceAction.set(jointSpacePAction.getDoubleValue() + jointSpaceDAction.getDoubleValue());

      return jointSpaceAction.getDoubleValue();
   }

   private double computeActuatorSpaceAction(double dampingActionScaleFactor)
   {
      double currentPosition = kneePitchJoint.getQ();
      double currentVelocity = kneePitchJoint.getQd();

      desiredVirtualActuatorLength.set(computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue()));
      currentVirtualActuatorLength.set(computeVirtualActuatorLength(currentPosition));
      currentVirtualActuatorVelocity.set(computeVirtualActuatorVelocity(currentPosition, currentVelocity));

      double virtualError = desiredVirtualActuatorLength.getDoubleValue() - currentVirtualActuatorLength.getDoubleValue();
      double actuatorSpaceKp = legConfigurationGains.hasActuatorSpaceKp() ? 0.0 : legConfigurationGains.getActuatorSpaceKp();
      double actuatorSpaceKd = legConfigurationGains.hasActuatorSpaceKd() ? 0.0 : dampingActionScaleFactor * legConfigurationGains.getActuatorSpaceKd();

      this.actuatorSpacePAction.set(actuatorSpaceKp * virtualError);
      this.actuatorSpaceDAction.set(actuatorSpaceKd * -currentVirtualActuatorVelocity.getDoubleValue());

      double actuatorSpaceAcceleration = actuatorSpacePAction.getDoubleValue() + actuatorSpaceDAction.getDoubleValue();

      double acceleration = computeActuatorAccelerationFromJointAcceleration(currentPosition, currentVelocity, actuatorSpaceAcceleration);

      this.actuatorSpaceAction.set(acceleration);

      return actuatorSpaceAction.getDoubleValue();
   }

   private double computeSpringSpaceAction(double dampingActionScaleFactor)
   {
      double currentPosition = kneePitchJoint.getQ();
      double currentVelocity = kneePitchJoint.getQd();


      desiredVirtualSpringLength.set(computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue()));
      currentVirtualSpringLength.set(computeVirtualActuatorLength(currentPosition));
      currentVirtualSpringVelocity.set(computeVirtualActuatorVelocity(currentPosition, currentVelocity));

      double virtualError = desiredVirtualSpringLength.getDoubleValue() - currentVirtualSpringLength.getDoubleValue();
      double springSpaceKp = legConfigurationGains.hasSpringSpaceKp() ? 0.0 : legConfigurationGains.getSpringSpaceKp();
      double springSpaceKd = legConfigurationGains.hasSpringSpaceKd() ? 0.0 : dampingActionScaleFactor * legConfigurationGains.getSpringSpaceKd();

      this.springSpacePAction.set(springSpaceKp * virtualError);
      this.springSpaceDAction.set(springSpaceKd * -currentVirtualSpringVelocity.getDoubleValue());

      double springSpaceAcceleration = springSpacePAction.getDoubleValue() + springSpaceDAction.getDoubleValue();

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
}
