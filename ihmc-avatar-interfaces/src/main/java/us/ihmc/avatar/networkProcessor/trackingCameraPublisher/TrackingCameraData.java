package us.ihmc.avatar.networkProcessor.trackingCameraPublisher;

import controller_msgs.msg.dds.TrackingCameraMessage;
import geometry_msgs.Point;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class TrackingCameraData
{
   private final long timestamp;
   private final double confidenceFactor;

   private final Point3D senorPosition;
   private final Quaternion sensorOrientation;

   private final Vector3D linearVelocity; // TODO: on TrackingCameraMessage. or StampedPosePacket.
   private final Vector3D angularVelocity;

   public TrackingCameraData(long timestamp)
   {
      this.timestamp = timestamp;
      this.confidenceFactor = 1.0;
      this.senorPosition = new Point3D();
      this.sensorOrientation = new Quaternion();
      this.linearVelocity = new Vector3D();
      this.angularVelocity = new Vector3D();
   }

   public void setPosition(Point position)
   {
      senorPosition.set(position.getX(), position.getY(), position.getZ());
   }

   public void setOrientation(geometry_msgs.Quaternion quaternion)
   {
      sensorOrientation.set(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public Point3D getSenorPosition()
   {
      return senorPosition;
   }

   public Quaternion getSensorOrientation()
   {
      return sensorOrientation;
   }

   public TrackingCameraMessage toTrackingCameraMessage()
   {
      return MessageTools.createTrackingCameraMessage(timestamp, confidenceFactor, senorPosition, sensorOrientation);
   }

   public void transform(RigidBodyTransform originWorldTransform)
   {
      originWorldTransform.transform(senorPosition);
      originWorldTransform.transform(sensorOrientation);
   }
}
