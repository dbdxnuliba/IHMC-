package us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CentroidalState implements CentroidalStateReadOnly
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFramePoint position;
   private final YoFrameVector linearVelocity;
   private final YoFrameVector linearAcceleration;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector angularVelocity;
   private final YoFrameVector angularAcceleration;

   public CentroidalState(String namePrefix, YoVariableRegistry registry)
   {
      position = new YoFramePoint(namePrefix + "Position", worldFrame, registry);
      linearVelocity = new YoFrameVector(namePrefix + "LinearVelocity", worldFrame, registry);
      linearAcceleration = new YoFrameVector(namePrefix + "LinearAcceleration", worldFrame, registry);
      orientation = new YoFrameQuaternion(namePrefix + "Orientation", worldFrame, registry);
      angularVelocity = new YoFrameVector(namePrefix + "AngularVelocity", worldFrame, registry);
      angularAcceleration = new YoFrameVector(namePrefix + "AngularAccleration", worldFrame, registry);
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
   public void getPosition(FramePoint3D positionToSet)
   {
      positionToSet.setIncludingFrame(position);
   }

   public void setPosition(FramePoint3D positionToSet)
   {
      position.set(positionToSet);
   }

   @Override
   public void getLinearVelocity(FrameVector3D linearVelocityToSet)
   {
      linearVelocityToSet.setIncludingFrame(linearVelocity);
   }

   public void setLinearVelocity(FrameVector3D linearVelocityToSet)
   {
      linearVelocity.set(linearVelocityToSet);
   }

   @Override
   public void getLinearAcceleration(FrameVector3D linearAccelerationToSet)
   {
      linearAccelerationToSet.setIncludingFrame(linearAcceleration);
   }

   public void setLinearAcceleration(FrameVector3D linearAccelerationToSet)
   {
      linearAcceleration.set(linearAccelerationToSet);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToSet)
   {
      orientationToSet.setIncludingFrame(orientation);
   }

   public void setOrientation(FrameQuaternion orientationToSet)
   {
      orientation.set(orientationToSet);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToSet)
   {
      angularVelocityToSet.setIncludingFrame(angularVelocity);
   }

   public void setAngularVelocity(FrameVector3D angularVelocityToSet)
   {
      angularVelocity.set(angularVelocityToSet);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToSet)
   {
      angularAccelerationToSet.setIncludingFrame(angularAcceleration);
   }

   public void setAngularAcceleration(FrameVector3D angularAccelerationToSet)
   {
      angularAcceleration.set(angularAccelerationToSet);
   }
}