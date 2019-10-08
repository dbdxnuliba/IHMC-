package us.ihmc.avatar.networkProcessor.trackingCameraPublisher;

import controller_msgs.msg.dds.TrackingCameraMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class TrackingCameraData
{
   private final long timestamp;
   private final double quality;
   private final Point3D senorPosition;
   private final Quaternion sensorOrientation;
   
   public TrackingCameraData(long timestamp, RigidBodyTransform transform)
   {
      this.timestamp = timestamp;
      this.quality = 0.0;
      this.senorPosition = new Point3D(transform.getTranslation());
      this.sensorOrientation = new Quaternion(transform.getRotation());
   }
   
   public long getTimestamp()
   {
      return timestamp;
   }
   
   public TrackingCameraMessage toTrackingCameraMessage()
   {
      return MessageTools.createTrackingCameraMessage(timestamp, 0.0, senorPosition, sensorOrientation);
   }
   
   // TODO: add transform tracking camera to depth camera
}
