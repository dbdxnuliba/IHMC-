package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface CentroidalStateReadOnly
{
   public FramePoint3DReadOnly getPosition();

   default public void getPosition(FramePoint3D positionToSet)
   {
      positionToSet.setIncludingFrame(getPosition());
   }

   public FrameVector3DReadOnly getLinearVelocity();

   default public void getLinearVelocity(FrameVector3D linearVelocityToSet)
   {
      linearVelocityToSet.setIncludingFrame(getLinearVelocity());
   }

   public FrameVector3DReadOnly getLinearAcceleration();

   default public void getLinearAcceleration(FrameVector3D linearAccelerationToSet)
   {
      linearAccelerationToSet.setIncludingFrame(getLinearAcceleration());
   }

   public FrameQuaternionReadOnly getOrientation();

   default public void getOrientation(FrameQuaternion orientationToSet)
   {
      orientationToSet.setIncludingFrame(getOrientation());
   }

   public FrameVector3DReadOnly getAngularVelocity();

   default public void getAngularVelocity(FrameVector3D angularVelocityToSet)
   {
      angularVelocityToSet.setIncludingFrame(getAngularVelocity());
   }

   public FrameVector3DReadOnly getAngularAcceleration();

   default public void getAngularAcceleration(FrameVector3D angularAccelerationToSet)
   {
      angularAccelerationToSet.setIncludingFrame(getAngularAcceleration());
   }
}