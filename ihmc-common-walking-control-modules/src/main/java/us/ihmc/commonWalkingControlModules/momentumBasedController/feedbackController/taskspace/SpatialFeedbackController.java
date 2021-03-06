package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.POSITION;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.ROTATION_VECTOR;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ACHIEVED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.CURRENT;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.DESIRED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR_CUMULATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR_INTEGRATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.FEEDBACK;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.FEEDFORWARD;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.math.filters.AlphaFilteredYoSpatialVector;
import us.ihmc.robotics.math.filters.RateLimitedYoSpatialVector;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SpatialFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoBoolean isEnabled;

   private final YoFramePose3D yoDesiredPose;
   private final YoFramePose3D yoCurrentPose;

   private final YoFixedFrameSpatialVector yoErrorVector;
   private final YoFrameQuaternion yoErrorOrientation;

   private final YoFrameVector3D yoErrorPositionIntegrated;
   private final YoFrameQuaternion yoErrorOrientationCumulated;
   private final YoFrameVector3D yoErrorRotationVectorIntegrated;

   private final YoFixedFrameSpatialVector yoDesiredVelocity;
   private final YoFixedFrameSpatialVector yoCurrentVelocity;
   private final YoFixedFrameSpatialVector yoErrorVelocity;
   private final AlphaFilteredYoSpatialVector yoFilteredErrorVelocity;
   private final YoFixedFrameSpatialVector yoFeedForwardVelocity;
   private final YoFixedFrameSpatialVector yoFeedbackVelocity;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackVelocity;

   private final YoFixedFrameSpatialVector yoDesiredAcceleration;
   private final YoFixedFrameSpatialVector yoFeedForwardAcceleration;
   private final YoFixedFrameSpatialVector yoFeedbackAcceleration;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackAcceleration;
   private final YoFixedFrameSpatialVector yoAchievedAcceleration;

   private final YoFixedFrameSpatialVector yoDesiredWrench;
   private final YoFixedFrameSpatialVector yoFeedForwardWrench;
   private final YoFixedFrameSpatialVector yoFeedbackWrench;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackWrench;

   private final YoFrameVector3D yoDesiredRotationVector;
   private final YoFrameVector3D yoCurrentRotationVector;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FramePose3D currentPose = new FramePose3D();
   private final FramePose3D desiredPose = new FramePose3D();

   private final FrameQuaternion errorOrientationCumulated = new FrameQuaternion();

   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final FrameVector3D currentAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularVelocity = new FrameVector3D();

   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAction = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAction = new FrameVector3D();
   private final FrameVector3D biasLinearAcceleration = new FrameVector3D();
   private final FrameVector3D achievedAngularAcceleration = new FrameVector3D();
   private final FrameVector3D achievedLinearAcceleration = new FrameVector3D();

   private final FrameVector3D desiredLinearForce = new FrameVector3D();
   private final FrameVector3D desiredAngularTorque = new FrameVector3D();

   private final Twist currentTwist = new Twist();
   private final SpatialAcceleration endEffectorAchievedAcceleration = new SpatialAcceleration();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();
   private final VirtualWrenchCommand virtualModelControlOutput = new VirtualWrenchCommand();
   private final MomentumRateCommand virtualModelControlRootOutput = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final YoPIDSE3Gains gains;
   private final YoPID3DGains positionGains;
   private final YoPID3DGains orientationGains;
   private final Matrix3D tempGainMatrix = new Matrix3D();

   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;

   private final RigidBodyBasics rootBody;
   private RigidBodyBasics base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame angularGainsFrame;
   private ReferenceFrame linearGainsFrame;

   private final RigidBodyBasics endEffector;
   private final YoSE3OffsetFrame controlFrame;

   private final double dt;
   private final boolean isRootBody;
   private final boolean computeIntegralTerm;

   public SpatialFeedbackController(RigidBodyBasics endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                    YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      FeedbackControllerSettings settings = toolbox.getFeedbackControllerSettings();
      if (settings != null)
         computeIntegralTerm = settings.enableIntegralTerm();
      else
         computeIntegralTerm = true;

      if (toolbox.getRootJoint() != null)
      {
         this.rootBody = toolbox.getRootJoint().getSuccessor();
         isRootBody = this.endEffector.getName().equals(rootBody.getName());
      }
      else
      {
         isRootBody = false;
         rootBody = null;
      }

      rigidBodyAccelerationProvider = toolbox.getRigidBodyAccelerationProvider();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "SpatialFBController");
      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getSE3PIDGains(endEffector, computeIntegralTerm);
      positionGains = gains.getPositionGains();
      orientationGains = gains.getOrientationGains();
      YoDouble maximumLinearRate = positionGains.getYoMaximumFeedbackRate();
      YoDouble maximumAngularRate = orientationGains.getYoMaximumFeedbackRate();

      controlFrame = feedbackControllerToolbox.getControlFrame(endEffector);

      isEnabled = new YoBoolean(endEffectorName + "isSpatialFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPose = feedbackControllerToolbox.getPose(endEffector, DESIRED, isEnabled);
      yoCurrentPose = feedbackControllerToolbox.getPose(endEffector, CURRENT, isEnabled);
      YoFrameVector3D errorPosition = feedbackControllerToolbox.getDataVector(endEffector, ERROR, POSITION, isEnabled);
      YoFrameVector3D errorRotationVector = feedbackControllerToolbox.getDataVector(endEffector, ERROR, ROTATION_VECTOR, isEnabled);
      yoErrorVector = new YoFixedFrameSpatialVector(errorRotationVector, errorPosition);
      yoErrorOrientation = feedbackControllerToolbox.getOrientation(endEffector, ERROR, isEnabled);
      yoErrorPositionIntegrated = computeIntegralTerm ? feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, POSITION, isEnabled) : null;
      yoErrorOrientationCumulated = computeIntegralTerm ? feedbackControllerToolbox.getOrientation(endEffector, ERROR_CUMULATED, isEnabled) : null;
      yoErrorRotationVectorIntegrated = computeIntegralTerm ? feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, ROTATION_VECTOR, isEnabled)
            : null;

      yoDesiredRotationVector = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ROTATION_VECTOR, isEnabled);
      yoCurrentRotationVector = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, ROTATION_VECTOR, isEnabled);

      yoDesiredVelocity = feedbackControllerToolbox.getVelocity(endEffector, DESIRED, isEnabled);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {

         yoCurrentVelocity = feedbackControllerToolbox.getVelocity(endEffector, CURRENT, isEnabled);
         yoErrorVelocity = feedbackControllerToolbox.getVelocity(endEffector, ERROR, isEnabled);

         DoubleProvider breakFrequency = feedbackControllerToolbox.getErrorVelocityFilterBreakFrequency(endEffectorName);
         if (breakFrequency != null)
            yoFilteredErrorVelocity = feedbackControllerToolbox.getAlphaFilteredVelocity(endEffector, ERROR, dt, breakFrequency, breakFrequency, isEnabled);
         else
            yoFilteredErrorVelocity = null;

         if (toolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, DESIRED, isEnabled);
            yoFeedForwardAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, FEEDFORWARD, isEnabled);
            yoFeedbackAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, FEEDBACK, isEnabled);
            rateLimitedFeedbackAcceleration = feedbackControllerToolbox.getRateLimitedAcceleration(endEffector, FEEDBACK, dt, maximumAngularRate,
                                                                                                   maximumLinearRate, isEnabled);
            yoAchievedAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, ACHIEVED, isEnabled);
         }
         else
         {
            yoDesiredAcceleration = null;
            yoFeedForwardAcceleration = null;
            yoFeedbackAcceleration = null;
            rateLimitedFeedbackAcceleration = null;
            yoAchievedAcceleration = null;
         }

         if (toolbox.isEnableVirtualModelControlModule())
         {
            yoDesiredWrench = feedbackControllerToolbox.getWrench(endEffector, DESIRED, isEnabled);
            yoFeedForwardWrench = feedbackControllerToolbox.getWrench(endEffector, FEEDFORWARD, isEnabled);
            yoFeedbackWrench = feedbackControllerToolbox.getWrench(endEffector, FEEDBACK, isEnabled);
            rateLimitedFeedbackWrench = feedbackControllerToolbox.getRateLimitedWrench(endEffector, FEEDBACK, dt, maximumAngularRate, maximumLinearRate,
                                                                                       isEnabled);
         }
         else
         {
            yoDesiredWrench = null;
            yoFeedForwardWrench = null;
            yoFeedbackWrench = null;
            rateLimitedFeedbackWrench = null;
         }
      }
      else
      {
         yoCurrentVelocity = null;
         yoErrorVelocity = null;
         yoFilteredErrorVelocity = null;

         yoDesiredAcceleration = null;
         yoFeedForwardAcceleration = null;
         yoFeedbackAcceleration = null;
         rateLimitedFeedbackAcceleration = null;
         yoAchievedAcceleration = null;

         yoDesiredWrench = null;
         yoFeedForwardWrench = null;
         yoFeedbackWrench = null;
         rateLimitedFeedbackWrench = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackVelocity = feedbackControllerToolbox.getVelocity(endEffector, FEEDBACK, isEnabled);
         yoFeedForwardVelocity = feedbackControllerToolbox.getVelocity(endEffector, FEEDFORWARD, isEnabled);
         rateLimitedFeedbackVelocity = feedbackControllerToolbox.getRateLimitedVelocity(endEffector, FEEDBACK, dt, maximumAngularRate, maximumLinearRate,
                                                                                        isEnabled);
      }
      else
      {
         yoFeedbackVelocity = null;
         yoFeedForwardVelocity = null;
         rateLimitedFeedbackVelocity = null;
      }

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(SpatialFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      controlBaseFrame = command.getControlBaseFrame();
      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());
      inverseKinematicsOutput.setProperties(command.getSpatialAccelerationCommand());
      virtualModelControlOutput.setProperties(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());
      command.getSpatialAccelerationCommand().getSelectionMatrix(selectionMatrix);
      angularGainsFrame = command.getAngularGainsFrame();
      linearGainsFrame = command.getLinearGainsFrame();

      command.getControlFramePoseIncludingFrame(desiredPosition, desiredOrientation);
      controlFrame.setOffsetToParent(desiredPosition, desiredOrientation);

      command.getIncludingFrame(desiredPosition, desiredLinearVelocity);
      command.getIncludingFrame(desiredOrientation, desiredAngularVelocity);
      command.getFeedForwardActionIncludingFrame(feedForwardAngularAction, feedForwardLinearAction);

      yoDesiredPose.setMatchingFrame(desiredPosition, desiredOrientation);
      yoDesiredPose.getOrientation().getRotationVector(yoDesiredRotationVector);
      yoDesiredVelocity.setMatchingFrame(desiredAngularVelocity, desiredLinearVelocity);

      if (yoFeedForwardVelocity != null)
         yoFeedForwardVelocity.setMatchingFrame(feedForwardAngularAction, feedForwardLinearAction);

      if (yoFeedForwardAcceleration != null)
         yoFeedForwardAcceleration.setMatchingFrame(feedForwardAngularAction, feedForwardLinearAction);

      if (yoFeedForwardWrench != null)
         yoFeedForwardWrench.setMatchingFrame(feedForwardAngularAction, feedForwardLinearAction);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      if (rateLimitedFeedbackAcceleration != null)
         rateLimitedFeedbackAcceleration.reset();
      if (rateLimitedFeedbackVelocity != null)
         rateLimitedFeedbackVelocity.reset();
      if (yoFilteredErrorVelocity != null)
         yoFilteredErrorVelocity.reset();
   }

   private final FrameVector3D linearProportionalFeedback = new FrameVector3D();
   private final FrameVector3D linearDerivativeFeedback = new FrameVector3D();
   private final FrameVector3D linearIntegralFeedback = new FrameVector3D();

   private final FrameVector3D angularProportionalFeedback = new FrameVector3D();
   private final FrameVector3D angularDerivativeFeedback = new FrameVector3D();
   private final FrameVector3D angularIntegralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeDerivativeTerm(linearDerivativeFeedback, angularDerivativeFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);
      feedForwardLinearAction.setIncludingFrame(yoFeedForwardAcceleration.getLinearPart());
      feedForwardAngularAction.setIncludingFrame(yoFeedForwardAcceleration.getAngularPart());
      feedForwardLinearAction.changeFrame(controlFrame);
      feedForwardAngularAction.changeFrame(controlFrame);

      desiredLinearAcceleration.setIncludingFrame(linearProportionalFeedback);
      desiredLinearAcceleration.add(linearDerivativeFeedback);
      desiredLinearAcceleration.add(linearIntegralFeedback);
      desiredLinearAcceleration.clipToMaxLength(positionGains.getMaximumFeedback());

      desiredAngularAcceleration.setIncludingFrame(angularProportionalFeedback);
      desiredAngularAcceleration.add(angularDerivativeFeedback);
      desiredAngularAcceleration.add(angularIntegralFeedback);
      desiredAngularAcceleration.clipToMaxLength(orientationGains.getMaximumFeedback());

      yoFeedbackAcceleration.setMatchingFrame(desiredAngularAcceleration, desiredLinearAcceleration);
      rateLimitedFeedbackAcceleration.update();
      desiredLinearAcceleration.setIncludingFrame(rateLimitedFeedbackAcceleration.getLinearPart());
      desiredAngularAcceleration.setIncludingFrame(rateLimitedFeedbackAcceleration.getAngularPart());

      desiredLinearAcceleration.changeFrame(controlFrame);
      desiredLinearAcceleration.add(feedForwardLinearAction);

      desiredAngularAcceleration.changeFrame(controlFrame);
      desiredAngularAcceleration.add(feedForwardAngularAction);

      yoDesiredAcceleration.setMatchingFrame(desiredAngularAcceleration, desiredLinearAcceleration);

      addCoriolisAcceleration(desiredLinearAcceleration);

      inverseDynamicsOutput.setSpatialAcceleration(controlFrame, desiredAngularAcceleration, desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);

      feedForwardLinearVelocity.setIncludingFrame(yoFeedForwardVelocity.getLinearPart());
      feedForwardAngularVelocity.setIncludingFrame(yoFeedForwardVelocity.getAngularPart());
      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);

      desiredLinearVelocity.setIncludingFrame(linearProportionalFeedback);
      desiredLinearVelocity.add(linearIntegralFeedback);
      desiredLinearVelocity.clipToMaxLength(positionGains.getMaximumFeedback());

      desiredAngularVelocity.setIncludingFrame(angularProportionalFeedback);
      desiredAngularVelocity.add(angularIntegralFeedback);
      desiredAngularVelocity.clipToMaxLength(orientationGains.getMaximumFeedback());

      yoFeedbackVelocity.setMatchingFrame(desiredAngularVelocity, desiredLinearVelocity);
      rateLimitedFeedbackVelocity.update();
      desiredLinearVelocity.setIncludingFrame(rateLimitedFeedbackVelocity.getLinearPart());
      desiredAngularVelocity.setIncludingFrame(rateLimitedFeedbackVelocity.getAngularPart());

      desiredLinearVelocity.add(feedForwardLinearVelocity);
      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredVelocity.setMatchingFrame(desiredAngularVelocity, desiredLinearVelocity);

      desiredLinearVelocity.changeFrame(controlFrame);
      desiredAngularVelocity.changeFrame(controlFrame);
      inverseKinematicsOutput.setSpatialVelocity(controlFrame, desiredAngularVelocity, desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      computeFeedbackWrench();

      if (isRootBody)
      {
         desiredLinearForce.changeFrame(worldFrame);
         desiredAngularTorque.changeFrame(worldFrame);

         virtualModelControlRootOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlRootOutput.setMomentumRate(desiredAngularTorque, desiredLinearForce);
      }
      else
      {
         virtualModelControlOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlOutput.setWrench(controlFrame, desiredAngularTorque, desiredLinearForce);
      }
   }

   private void computeFeedbackWrench()
   {
      feedForwardLinearAction.setIncludingFrame(yoFeedForwardWrench. getLinearPart());
      feedForwardAngularAction.setIncludingFrame(yoFeedForwardWrench.getAngularPart());
      feedForwardLinearAction.changeFrame(controlFrame);
      feedForwardAngularAction.changeFrame(controlFrame);
      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeDerivativeTerm(linearDerivativeFeedback, angularDerivativeFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);

      desiredLinearForce.setIncludingFrame(linearProportionalFeedback);
      desiredLinearForce.add(linearDerivativeFeedback);
      desiredLinearForce.add(linearIntegralFeedback);
      desiredLinearForce.clipToMaxLength(positionGains.getMaximumFeedback());

      desiredAngularTorque.setIncludingFrame(angularProportionalFeedback);
      desiredAngularTorque.add(angularDerivativeFeedback);
      desiredAngularTorque.add(angularIntegralFeedback);
      desiredAngularTorque.clipToMaxLength(orientationGains.getMaximumFeedback());

      yoFeedbackWrench.setMatchingFrame(desiredAngularTorque, desiredLinearForce);
      rateLimitedFeedbackWrench.update();
      desiredLinearForce  .setIncludingFrame(rateLimitedFeedbackWrench.getLinearPart());
      desiredAngularTorque.setIncludingFrame(rateLimitedFeedbackWrench.getAngularPart());

      desiredLinearForce.changeFrame(controlFrame);
      desiredLinearForce.add(feedForwardLinearAction);

      desiredAngularTorque.changeFrame(controlFrame);
      desiredAngularTorque.add(feedForwardAngularAction);

      yoDesiredWrench.setMatchingFrame(desiredAngularTorque, desiredLinearForce);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      endEffectorAchievedAcceleration.setIncludingFrame(rigidBodyAccelerationProvider.getRelativeAcceleration(base, endEffector));
      endEffectorAchievedAcceleration.changeFrame(controlFrame);
      achievedAngularAcceleration.setIncludingFrame(endEffectorAchievedAcceleration.getAngularPart());
      achievedLinearAcceleration.setIncludingFrame(endEffectorAchievedAcceleration.getLinearPart());
      subtractCoriolisAcceleration(achievedLinearAcceleration);

      yoAchievedAcceleration.getAngularPart().setMatchingFrame(achievedAngularAcceleration);
      yoAchievedAcceleration.getLinearPart().setMatchingFrame(achievedLinearAcceleration);
   }

   /**
    * Computes the feedback term resulting from the error in position and orienation:<br>
    * x<sub>FB</sub><sup>linear</sup> = kp<sup>linear</sup> * (x<sub>desired</sub> -
    * x<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = kp<sup>angular</sup> * &theta;<sub>error</sub><br>
    * where &theta;<sub>error</sub> is a rotation vector representing the current error in orientation.
    * <p>
    * The desired pose of the {@code controlFrame} is obtained from {@link #yoDesiredPose}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentPose}, {@link #yoErrorVector}, and
    * {@link #yoErrorOrientation}.
    * </p>
    *
    * @param linearFeedbackTermToPack the value of the feedback term x<sub>FB</sub><sup>linear</sup>.
    *           Modified.
    * @param angularFeedbackTermToPack the value of the feedback term x<sub>FB</sub><sup>angular</sup>.
    *           Modified.
    */
   private void computeProportionalTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      currentPose.setToZero(controlFrame);
      currentPose.changeFrame(worldFrame);
      yoCurrentPose.set(currentPose);
      yoCurrentPose.getOrientation().getRotationVector(yoCurrentRotationVector);

      desiredPose.setIncludingFrame(yoDesiredPose);
      desiredPose.changeFrame(controlFrame);

      desiredPose.normalizeQuaternionAndLimitToPi();
      linearFeedbackTermToPack.setIncludingFrame(desiredPose.getPosition());
      desiredPose.getRotationVector(angularFeedbackTermToPack);

      selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
      selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);

      linearFeedbackTermToPack.clipToMaxLength(positionGains.getMaximumProportionalError());
      angularFeedbackTermToPack.clipToMaxLength(orientationGains.getMaximumProportionalError());

      yoErrorVector.setMatchingFrame(angularFeedbackTermToPack, linearFeedbackTermToPack);
      yoErrorOrientation.setRotationVector(yoErrorVector.getAngularPart());

      if (linearGainsFrame != null)
         linearFeedbackTermToPack.changeFrame(linearGainsFrame);
      else
         linearFeedbackTermToPack.changeFrame(controlFrame);

      if (angularGainsFrame != null)
         angularFeedbackTermToPack.changeFrame(angularGainsFrame);
      else
         angularFeedbackTermToPack.changeFrame(controlFrame);

      positionGains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(linearFeedbackTermToPack);

      orientationGains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(angularFeedbackTermToPack);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the error in linear velocity:<br>
    * x<sub>FB</sub><sup>linear</sup> = kd<sup>linear</sup> * (xDot<sub>desired</sub> -
    * xDot<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = kd<sup>angular</sup> * (&omega;<sub>desired</sub> -
    * &omega;<sub>current</sub>)
    * <p>
    * The desired velocity of the {@code controlFrame} relative to the {@code base} is obtained from
    * {@link #yoDesiredVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentVelocity} and {@link #yoErrorVelocity}.
    * </p>
    *
    * @param linearFeedbackTermToPack the value of the feedback term x<sub>FB</sub><sup>linear</sup>.
    *           Modified.
    * @param angularFeedbackTermToPack the value of the feedback term x<sub>FB</sub><sup>angular</sup>.
    *           Modified.
    */
   private void computeDerivativeTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      currentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());
      currentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      currentLinearVelocity.changeFrame(worldFrame);
      currentAngularVelocity.changeFrame(worldFrame);
      yoCurrentVelocity.setMatchingFrame(currentAngularVelocity, currentLinearVelocity);

      desiredLinearVelocity.setIncludingFrame(yoDesiredVelocity.getLinearPart());
      desiredAngularVelocity.setIncludingFrame(yoDesiredVelocity.getAngularPart());

      linearFeedbackTermToPack.setToZero(worldFrame);
      angularFeedbackTermToPack.setToZero(worldFrame);
      linearFeedbackTermToPack.sub(desiredLinearVelocity, currentLinearVelocity);
      angularFeedbackTermToPack.sub(desiredAngularVelocity, currentAngularVelocity);
      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
      selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);

      linearFeedbackTermToPack.clipToMaxLength(positionGains.getMaximumDerivativeError());
      angularFeedbackTermToPack.clipToMaxLength(orientationGains.getMaximumDerivativeError());

      if (yoFilteredErrorVelocity != null)
      {
         linearFeedbackTermToPack.changeFrame(worldFrame);
         angularFeedbackTermToPack.changeFrame(worldFrame);
         yoErrorVelocity.set(angularFeedbackTermToPack, linearFeedbackTermToPack);
         yoFilteredErrorVelocity.update();
         linearFeedbackTermToPack.set(yoFilteredErrorVelocity.getLinearPart());
         angularFeedbackTermToPack.set(yoFilteredErrorVelocity.getAngularPart());
      }
      else
      {
         yoErrorVelocity.setMatchingFrame(angularFeedbackTermToPack, linearFeedbackTermToPack);
      }

      if (linearGainsFrame != null)
         linearFeedbackTermToPack.changeFrame(linearGainsFrame);
      else
         linearFeedbackTermToPack.changeFrame(controlFrame);

      if (angularGainsFrame != null)
         angularFeedbackTermToPack.changeFrame(angularGainsFrame);
      else
         angularFeedbackTermToPack.changeFrame(controlFrame);

      positionGains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(linearFeedbackTermToPack);

      orientationGains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(angularFeedbackTermToPack);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the integrated error in position:<br>
    * x<sub>FB</sub><sup>linear</sup> = ki<sup>linear</sup> * &int;<sup>t</sup> (x<sub>desired</sub> -
    * x<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = ki<sup>angular</sup> * &int;<sup>t</sup>
    * &theta;<sub>error</sub>
    * <p>
    * The current error in position and orientation of the {@code controlFrame} is obtained from
    * {@link #yoErrorVector} and {@link #yoErrorOrientation} respectively.
    * </p>
    * <p>
    * This method also updates {@link #yoErrorPositionIntegrated},
    * {@link #yoErrorOrientationCumulated}, and {@link #yoErrorRotationVectorIntegrated}.
    * </p>
    *
    * @param linearFeedbackTermToPack the value of the feedback term x<sub>FB</sub><sup>linear</sup>.
    *           Modified.
    * @param angularFeedbackTermToPack the value of the feedback term x<sub>FB</sub><sup>angular</sup>.
    *           Modified.
    */
   private void computeIntegralTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      if (!computeIntegralTerm)
      {
         linearFeedbackTermToPack.setToZero(controlFrame);
         angularFeedbackTermToPack.setToZero(controlFrame);
         return;
      }

      double maximumLinearIntegralError = positionGains.getMaximumIntegralError();

      if (maximumLinearIntegralError < 1.0e-5)
      {
         linearFeedbackTermToPack.setToZero(controlFrame);
         yoErrorPositionIntegrated.setToZero();
      }
      else
      {
         linearFeedbackTermToPack.setIncludingFrame(yoErrorVector.getLinearPart());
         linearFeedbackTermToPack.scale(dt);
         linearFeedbackTermToPack.add(yoErrorPositionIntegrated);
         linearFeedbackTermToPack.changeFrame(controlFrame);
         selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
         linearFeedbackTermToPack.clipToMaxLength(maximumLinearIntegralError);
         yoErrorPositionIntegrated.setMatchingFrame(linearFeedbackTermToPack);

         if (linearGainsFrame != null)
            linearFeedbackTermToPack.changeFrame(linearGainsFrame);
         else
            linearFeedbackTermToPack.changeFrame(controlFrame);

         positionGains.getIntegralGainMatrix(tempGainMatrix);
         tempGainMatrix.transform(linearFeedbackTermToPack);

         linearFeedbackTermToPack.changeFrame(controlFrame);
      }

      double maximumAngularIntegralError = orientationGains.getMaximumIntegralError();

      if (maximumAngularIntegralError < 1.0e-5)
      {
         angularFeedbackTermToPack.setToZero(controlFrame);
         yoErrorOrientationCumulated.setToZero();
         yoErrorRotationVectorIntegrated.setToZero();
      }
      else
      {
         errorOrientationCumulated.setIncludingFrame(yoErrorOrientationCumulated);
         errorOrientationCumulated.multiply(yoErrorOrientation);
         yoErrorOrientationCumulated.set(errorOrientationCumulated);
         errorOrientationCumulated.normalizeAndLimitToPi();

         errorOrientationCumulated.getRotationVector(angularFeedbackTermToPack);
         angularFeedbackTermToPack.scale(dt);
         angularFeedbackTermToPack.changeFrame(controlFrame);
         selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);
         angularFeedbackTermToPack.clipToMaxLength(maximumAngularIntegralError);
         yoErrorRotationVectorIntegrated.setMatchingFrame(angularFeedbackTermToPack);

         if (angularGainsFrame != null)
            angularFeedbackTermToPack.changeFrame(angularGainsFrame);
         else
            angularFeedbackTermToPack.changeFrame(controlFrame);

         orientationGains.getIntegralGainMatrix(tempGainMatrix);
         tempGainMatrix.transform(angularFeedbackTermToPack);

         angularFeedbackTermToPack.changeFrame(controlFrame);
      }
   }

   /**
    * Computes and adds the bias acceleration resulting from the combination of the current linear and
    * angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in an inertial frame to a moving
    * frame attached to the end-effector.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a moving
    * frame, here a frame attache to the end-effector.
    * </p>
    *
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *           subtracted. Its frame is changed to {@code controlFrame}. Modified.
    */
   private void addCoriolisAcceleration(FrameVector3D linearAccelerationToModify)
   {
      controlFrame.getTwistOfFrame(currentTwist);
      currentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      currentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.add(biasLinearAcceleration);
   }

   /**
    * Computes and subtracts the bias acceleration resulting from the combination of the current linear
    * and angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in a moving frame attached to the
    * end-effector to an inertial frame.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a moving
    * frame, here a frame attache to the end-effector.
    * </p>
    *
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *           added. Its frame is changed to {@code worldFrame}. Modified.
    */
   private void subtractCoriolisAcceleration(FrameVector3D linearAccelerationToModify)
   {
      ReferenceFrame originalFrame = linearAccelerationToModify.getReferenceFrame();
      controlFrame.getTwistOfFrame(currentTwist);
      currentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      currentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.sub(biasLinearAcceleration);
      linearAccelerationToModify.changeFrame(originalFrame);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public SpatialAccelerationCommand getInverseDynamicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public SpatialVelocityCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return (isRootBody) ? virtualModelControlRootOutput : virtualModelControlOutput;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": endEffector = " + endEffector;
   }
}
