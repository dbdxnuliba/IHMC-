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
   private static final double minKneeAngleToAvoidAccelExplosion = 0.7;

   private static final double minimumDampingScale = 0.2;
   private static final boolean scaleDamping = false;

   private final LegConfigurationControlToolbox toolbox;
   private final OneDoFJointBasics kneePitchJoint;

   private final YoDouble privilegedAcceleration;
   private final YoDouble privilegedMaxAcceleration;
   private final YoDouble dampingActionScaleFactor;

   private final YoDouble kneePitchPrivilegedConfiguration;

   private final YoDouble jointSpacePAction;
   private final YoDouble jointSpaceDAction;
   private final YoDouble jointSpaceJointAction;

   private final YoDouble actuatorSpacePAction;
   private final YoDouble actuatorSpaceDAction;
   private final YoDouble actuatorSpaceAction;
   private final YoDouble actuatorSpaceJointAction;

   private final YoDouble springSpacePAction;
   private final YoDouble springSpaceDAction;
   private final YoDouble springSpaceAction;
   private final YoDouble springSpaceJointAction;

   private final YoDouble jointSpaceError;
   private final YoDouble legLengthError;

   private final YoDouble desiredLegLength;
   private final YoDouble currentLegLength;
   private final YoDouble currentLegVelocity;

   private final double thighLength;
   private final double shinLength;

   private LegConfigurationGainsReadOnly legConfigurationGains;

   public LegConfigurationController(String sidePrefix, LegConfigurationControlToolbox toolbox, FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      this.kneePitchJoint = toolbox.getKneePitchJoint();

      String namePrefix = sidePrefix + "Leg";

      kneePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "KneePitchConfiguration", registry);

      privilegedAcceleration = new YoDouble(sidePrefix + "LegPrivilegedAcceleration", registry);
      privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);
      dampingActionScaleFactor = new YoDouble(namePrefix + "DampingActionScaleFactor", registry);

      jointSpacePAction = new YoDouble(sidePrefix + "KneeJointSpacePAction", registry);
      jointSpaceDAction = new YoDouble(sidePrefix + "KneeJointSpaceDAction", registry);
      jointSpaceJointAction = new YoDouble(sidePrefix + "KneeJointSpaceJointAction", registry);

      actuatorSpacePAction = new YoDouble(sidePrefix + "KneeActuatorSpacePAction", registry);
      actuatorSpaceDAction = new YoDouble(sidePrefix + "KneeActuatorSpaceDAction", registry);
      actuatorSpaceAction = new YoDouble(sidePrefix + "KneeActuatorSpaceAction", registry);
      actuatorSpaceJointAction = new YoDouble(sidePrefix + "KneeActuatorSpaceJointAction", registry);

      springSpacePAction = new YoDouble(sidePrefix + "KneeSpringSpacePAction", registry);
      springSpaceDAction = new YoDouble(sidePrefix + "KneeSpringSpaceDAction", registry);
      springSpaceAction = new YoDouble(sidePrefix + "KneeSpringSpaceAction", registry);
      springSpaceJointAction = new YoDouble(sidePrefix + "KneeSpringSpaceJointAction", registry);

      legLengthError = new YoDouble(namePrefix + "LegLengthError", registry);
      jointSpaceError = new YoDouble(namePrefix + "JointSpaceError", registry);

      desiredLegLength = new YoDouble(namePrefix + "DesiredLegLength", registry);
      currentLegLength = new YoDouble(namePrefix + "CurrentLegLength", registry);
      currentLegVelocity = new YoDouble(namePrefix + "CurrentLegVelocity", registry);

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

   public void computeKneeAcceleration()
   {
      double currentPosition = kneePitchJoint.getQ();

      double jointError = kneePitchPrivilegedConfiguration.getDoubleValue() - currentPosition;

      // modify gains based on error. If there's a big error, don't damp velocities
      double percentError = Math.abs(jointError) / (0.5 * toolbox.getKneeRangeOfMotion());
      double dampingActionScaleFactor = scaleDamping ? MathTools.clamp(1.0 - (1.0 - minimumDampingScale) * percentError, 0.0, 1.0) : 1.0;
      this.dampingActionScaleFactor.set(dampingActionScaleFactor);

      updateLegLengths();

      double jointSpaceAction = computeJointSpaceAction();
      double actuatorSpaceAction = computeActuatorSpaceAction();
      double springSpaceAction = computeSpringSpaceAction();

      double desiredAcceleration = jointSpaceAction + actuatorSpaceAction + springSpaceAction;

      privilegedAcceleration.set(MathTools.clamp(desiredAcceleration, privilegedMaxAcceleration.getDoubleValue()));
   }

   public double getKneeAcceleration()
   {
      return privilegedAcceleration.getDoubleValue();
   }

   private void updateLegLengths()
   {
      desiredLegLength.set(computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue()));
      currentLegLength.set(computeVirtualActuatorLength(kneePitchJoint.getQ()));
      currentLegVelocity.set(computeVirtualActuatorVelocity(kneePitchJoint.getQ(), kneePitchJoint.getQd()));

      legLengthError.set(desiredLegLength.getDoubleValue() - currentLegLength.getDoubleValue());
   }

   private double computeJointSpaceAction()
   {
      jointSpaceError.set(kneePitchPrivilegedConfiguration.getDoubleValue() - kneePitchJoint.getQ());
      double jointSpaceKp = legConfigurationGains.hasJointSpaceKp() ? 2.0 * legConfigurationGains.getJointSpaceKp() / toolbox.getKneeSquareRangeOfMotion() : 0.0;
      double jointSpaceKd = legConfigurationGains.hasJointSpaceKd() ? dampingActionScaleFactor.getDoubleValue() * legConfigurationGains.getJointSpaceKd() : 0.0;

      jointSpacePAction.set(jointSpaceKp * jointSpaceError.getDoubleValue());
      jointSpaceDAction.set(jointSpaceKd * -kneePitchJoint.getQd());
      jointSpaceJointAction.set(jointSpacePAction.getDoubleValue() + jointSpaceDAction.getDoubleValue());

      return jointSpaceJointAction.getDoubleValue();
   }


   private double computeActuatorSpaceAction()
   {
      double actuatorSpaceKp = legConfigurationGains.hasActuatorSpaceKp() ? legConfigurationGains.getActuatorSpaceKp() : 0.0;
      double actuatorSpaceKd = legConfigurationGains.hasActuatorSpaceKd() ? dampingActionScaleFactor.getDoubleValue() * legConfigurationGains.getActuatorSpaceKd() : 0.0;

      actuatorSpacePAction.set(actuatorSpaceKp * legLengthError.getDoubleValue());
      actuatorSpaceDAction.set(actuatorSpaceKd * -currentLegVelocity.getDoubleValue());
      actuatorSpaceAction.set(actuatorSpacePAction.getDoubleValue() + actuatorSpaceDAction.getDoubleValue());

      actuatorSpaceJointAction.set(computeActuatorAccelerationFromJointAcceleration(kneePitchJoint.getQ(), kneePitchJoint.getQd(), actuatorSpaceAction.getDoubleValue()));
      return actuatorSpaceJointAction.getDoubleValue();
   }

   private double computeSpringSpaceAction()
   {
      double springSpaceKp = legConfigurationGains.hasSpringSpaceKp() ? legConfigurationGains.getSpringSpaceKp() : 0.0;
      double springSpaceKd = legConfigurationGains.hasSpringSpaceKd() ? dampingActionScaleFactor.getDoubleValue() * legConfigurationGains.getSpringSpaceKd() : 0.0;

      springSpacePAction.set(springSpaceKp * legLengthError.getDoubleValue());
      springSpaceDAction.set(springSpaceKd * -currentLegVelocity.getDoubleValue());
      springSpaceAction.set(springSpacePAction.getDoubleValue() + springSpaceDAction.getDoubleValue());

      springSpaceJointAction.set(computeKneeAccelerationFromLegAcceleration(kneePitchJoint.getQ(), kneePitchJoint.getQd(), springSpaceAction.getDoubleValue()));
      return springSpaceJointAction.getDoubleValue();
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
      double interiorAngle = Math.PI - Math.max(kneePitchAngle, minKneeAngle);
      double interiorVelocity = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchVelocity) : -kneePitchVelocity;
      double interiorAcceleration = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchAcceleration) : -kneePitchAcceleration;

      return TriangleTools.computeSideLengthAcceleration(thighLength, shinLength, currentLegLength.getDoubleValue(), currentLegVelocity.getDoubleValue(),
                                                         interiorAngle, interiorVelocity, interiorAcceleration);
   }

   private double computeKneeAccelerationFromLegAcceleration(double kneePitchAngle, double kneePitchVelocity, double actuatorAcceleration)
   {
      double interiorAngle = Math.PI - Math.max(Math.max(kneePitchAngle, minKneeAngle), minKneeAngleToAvoidAccelExplosion);
      double interiorVelocity = kneePitchAngle < minKneeAngle ? -Math.max(0.0, kneePitchVelocity) : -kneePitchVelocity;

      return -TriangleTools.computeInteriorAngleAcceleration(thighLength, shinLength, currentLegLength.getDoubleValue(), currentLegVelocity.getDoubleValue(),
                                                             actuatorAcceleration, interiorAngle, interiorVelocity);
   }
}
