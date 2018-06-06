package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LinearMotionController
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameVector linearMomentumProportionalGains;
   private final YoFrameVector linearMomentumDerivativeGains;

   private final YoFramePoint estimatedCoM;
   private final YoFramePoint desiredCoM;
   private final YoFrameVector estimatedCoMVelocity;
   private final YoFrameVector desiredCoMVelocity;
   private final YoFrameVector feedforwardLinearAcceleration;
   private final YoFrameVector proportionalFeedback;
   private final YoFrameVector derivativeFeedback;
   private final YoFrameVector feedbackLinearAcceleration;
   private final YoFrameVector linearAccelerationCommand;

   public LinearMotionController(YoVariableRegistry registry)
   {
      String namePrefix = "CenterOfMassController";
      linearMomentumProportionalGains = new YoFrameVector(namePrefix + "LinearProportionalGain", worldFrame, registry);
      linearMomentumDerivativeGains = new YoFrameVector(namePrefix + "LinearDerivativeGain", worldFrame, registry);

      desiredCoM = new YoFramePoint(namePrefix + "DesiredPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      estimatedCoM = new YoFramePoint(namePrefix + "EstimatedPosition", worldFrame, registry);
      estimatedCoMVelocity = new YoFrameVector(namePrefix + "EstimatedLinearVelocity", worldFrame, registry);
      feedforwardLinearAcceleration = new YoFrameVector(namePrefix + "FeedforwardAcceleration", worldFrame, registry);
      feedbackLinearAcceleration = new YoFrameVector(namePrefix + "FeedbackAcceleration", worldFrame, registry);
      linearAccelerationCommand = new YoFrameVector(namePrefix + "LinearAccelerationCommand", worldFrame, registry);

      proportionalFeedback = new YoFrameVector(namePrefix + "ProportionalFeedback", worldFrame, registry);
      derivativeFeedback = new YoFrameVector(namePrefix + "DerivativeFeedback", worldFrame, registry);
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
}