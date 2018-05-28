package us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface CentroidalStateReadOnly
{
   void getPosition(FramePoint3D positionToSet);

   void getLinearVelocity(FrameVector3D linearVelocityToSet);

   void getLinearAcceleration(FrameVector3D linearAccelerationToSet);

   void getOrientation(FrameQuaternion orientationToSet);

   void getAngularVelocity(FrameVector3D angularVelocityToSet);

   void getAngularAcceleration(FrameVector3D angularAccelerationToSet);
}
