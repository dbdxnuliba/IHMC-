package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class LinearMotionController
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector3D linearMomentumProportionalGains;
   private final YoFrameVector3D linearMomentumDerivativeGains;

   private final YoFramePoint3D estimatedCoM;
   private final YoFramePoint3D desiredCoM;
   private final YoFrameVector3D estimatedCoMVelocity;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D feedforwardLinearAcceleration;
   private final YoFrameVector3D proportionalFeedback;
   private final YoFrameVector3D derivativeFeedback;
   private final YoFrameVector3D feedbackLinearAcceleration;
   private final YoFrameVector3D linearAccelerationCommand;

   public LinearMotionController(YoVariableRegistry registry)
   {
      String namePrefix = "CenterOfMassController";
      linearMomentumProportionalGains = new YoFrameVector3D(namePrefix + "LinearProportionalGain", worldFrame, registry);
      linearMomentumDerivativeGains = new YoFrameVector3D(namePrefix + "LinearDerivativeGain", worldFrame, registry);

      desiredCoM = new YoFramePoint3D(namePrefix + "DesiredPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      estimatedCoM = new YoFramePoint3D(namePrefix + "EstimatedPosition", worldFrame, registry);
      estimatedCoMVelocity = new YoFrameVector3D(namePrefix + "EstimatedLinearVelocity", worldFrame, registry);
      feedforwardLinearAcceleration = new YoFrameVector3D(namePrefix + "FeedforwardAcceleration", worldFrame, registry);
      feedbackLinearAcceleration = new YoFrameVector3D(namePrefix + "FeedbackAcceleration", worldFrame, registry);
      linearAccelerationCommand = new YoFrameVector3D(namePrefix + "LinearAccelerationCommand", worldFrame, registry);

      proportionalFeedback = new YoFrameVector3D(namePrefix + "ProportionalFeedback", worldFrame, registry);
      derivativeFeedback = new YoFrameVector3D(namePrefix + "DerivativeFeedback", worldFrame, registry);
   }

   public void setLinearMomentumFeedbackGains(FrameVector3DReadOnly linearProportionalGain, FrameVector3DReadOnly linearDerivativeGain)
   {
      linearMomentumProportionalGains.set(linearProportionalGain);
      linearMomentumDerivativeGains.set(linearDerivativeGain);
   }

   public void setPlannedCoM(FramePoint3DReadOnly plannedCoMPosition, FrameVector3DReadOnly plannedCoMVelocity)
   {
      desiredCoM.set(plannedCoMPosition);
      desiredCoMVelocity.set(plannedCoMVelocity);
   }

   public void setEstimatedCoM(FramePoint3DReadOnly currentCoMPosition, FrameVector3DReadOnly currentCoMVelocity)
   {
      estimatedCoM.set(currentCoMPosition);
      estimatedCoMVelocity.set(currentCoMVelocity);
   }

   public void setFeedforwardLinearAcceleration(FrameVector3DReadOnly feedforwardAcceleration)
   {
      feedforwardLinearAcceleration.set(feedforwardAcceleration);
   }

   public void doControl()
   {
      proportionalFeedback.sub(desiredCoM, estimatedCoM);
      proportionalFeedback.scale(linearMomentumProportionalGains.getX(), linearMomentumProportionalGains.getY(), linearMomentumProportionalGains.getZ());

      derivativeFeedback.sub(desiredCoMVelocity, estimatedCoMVelocity);
      derivativeFeedback.scale(linearMomentumDerivativeGains.getX(), linearMomentumDerivativeGains.getY(), linearMomentumProportionalGains.getZ());
      
      feedbackLinearAcceleration.add(proportionalFeedback, derivativeFeedback);
      linearAccelerationCommand.add(feedbackLinearAcceleration, feedforwardLinearAcceleration);
   }

   public FrameVector3DReadOnly getLinearAccelerationCommand()
   {
      return linearAccelerationCommand;
   }
}