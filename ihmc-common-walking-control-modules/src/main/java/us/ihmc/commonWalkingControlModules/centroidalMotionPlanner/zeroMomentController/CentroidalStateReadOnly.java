package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface CentroidalStateReadOnly
{
   default void getPosition(FramePoint3D positionToSet)
   {
      positionToSet.setIncludingFrame(getPosition());
   }

   FramePoint3DReadOnly getPosition();

   default void getLinearVelocity(FrameVector3D linearVelocityToSet)
   {
      linearVelocityToSet.setIncludingFrame(getLinearVelocity());
   }

   FrameVector3DReadOnly getLinearVelocity();

   default void getLinearAcceleration(FrameVector3D linearAccelerationToSet)
   {
      linearAccelerationToSet.setIncludingFrame(getLinearAcceleration());
   }

   FrameVector3DReadOnly getLinearAcceleration();

   default void getOrientation(FrameQuaternion orientationToSet)
   {
      orientationToSet.setIncludingFrame(getOrientation());
   }

   FrameQuaternionReadOnly getOrientation();

   default void getAngularVelocity(FrameVector3D angularVelocityToSet)
   {
      angularVelocityToSet.setIncludingFrame(getAngularVelocity());
   }

   FrameVector3DReadOnly getAngularVelocity();

   default void getAngularAcceleration(FrameVector3D angularAccelerationToSet)
   {
      angularAccelerationToSet.setIncludingFrame(getAngularAcceleration());
   }

   FrameVector3DReadOnly getAngularAcceleration();
}
