package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class CentroidalState implements CentroidalStateReadOnly
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFramePoint3D position;
   private final YoFrameVector3D linearVelocity;
   private final YoFrameVector3D linearAcceleration;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector3D angularVelocity;
   private final YoFrameVector3D angularAcceleration;

   public CentroidalState(String namePrefix, YoVariableRegistry registry)
   {
      position = new YoFramePoint3D(namePrefix + "Position", worldFrame, registry);
      linearVelocity = new YoFrameVector3D(namePrefix + "LinearVelocity", worldFrame, registry);
      linearAcceleration = new YoFrameVector3D(namePrefix + "LinearAcceleration", worldFrame, registry);
      orientation = new YoFrameQuaternion(namePrefix + "Orientation", worldFrame, registry);
      angularVelocity = new YoFrameVector3D(namePrefix + "AngularVelocity", worldFrame, registry);
      angularAcceleration = new YoFrameVector3D(namePrefix + "AngularAccleration", worldFrame, registry);
   }

   public void set(CentroidalState other)
   {
      position.set(other.position);
      linearVelocity.set(other.linearVelocity);
      linearAcceleration.set(other.linearAcceleration);
      orientation.set(other.orientation);
      angularVelocity.set(other.angularVelocity);
      angularAcceleration.set(other.angularAcceleration);
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   public void setPosition(FramePoint3D positionToSet)
   {
      position.set(positionToSet);
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }

   public void setLinearVelocity(FrameVector3D linearVelocityToSet)
   {
      linearVelocity.set(linearVelocityToSet);
   }

   @Override
   public FrameVector3DReadOnly getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public void setLinearAcceleration(FrameVector3D linearAccelerationToSet)
   {
      linearAcceleration.set(linearAccelerationToSet);
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   public void setOrientation(FrameQuaternion orientationToSet)
   {
      orientation.set(orientationToSet);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   public void setAngularVelocity(FrameVector3D angularVelocityToSet)
   {
      angularVelocity.set(angularVelocityToSet);
   }

   @Override
   public FrameVector3DReadOnly getAngularAcceleration()
   {
      return angularAcceleration;
   }

   public void setAngularAcceleration(FrameVector3D angularAccelerationToSet)
   {
      angularAcceleration.set(angularAccelerationToSet);
   }
}