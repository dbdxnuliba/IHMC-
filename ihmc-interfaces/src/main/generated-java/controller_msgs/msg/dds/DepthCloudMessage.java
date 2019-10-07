package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Should disappear for the ROS equivalent.
       */
public class DepthCloudMessage extends Packet<DepthCloudMessage> implements Settable<DepthCloudMessage>, EpsilonComparable<DepthCloudMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public long timestamp_;
   public us.ihmc.euclid.tuple3D.Point3D sensor_position_;
   public us.ihmc.euclid.tuple4D.Quaternion sensor_orientation_;
   public us.ihmc.euclid.tuple3D.Vector3D linear_velocity_;
   public us.ihmc.euclid.tuple3D.Vector3D angular_velocity_;
   public us.ihmc.idl.IDLSequence.Float  point_cloud_;

   public DepthCloudMessage()
   {
      sensor_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      sensor_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      point_cloud_ = new us.ihmc.idl.IDLSequence.Float (600000, "type_5");

   }

   public DepthCloudMessage(DepthCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(DepthCloudMessage other)
   {
      sequence_id_ = other.sequence_id_;

      timestamp_ = other.timestamp_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.sensor_position_, sensor_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.sensor_orientation_, sensor_orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_velocity_, linear_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_velocity_, angular_velocity_);
      point_cloud_.set(other.point_cloud_);
   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getSensorPosition()
   {
      return sensor_position_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getSensorOrientation()
   {
      return sensor_orientation_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getLinearVelocity()
   {
      return linear_velocity_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getAngularVelocity()
   {
      return angular_velocity_;
   }


   public us.ihmc.idl.IDLSequence.Float  getPointCloud()
   {
      return point_cloud_;
   }


   public static Supplier<DepthCloudMessagePubSubType> getPubSubType()
   {
      return DepthCloudMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DepthCloudMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DepthCloudMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!this.sensor_position_.epsilonEquals(other.sensor_position_, epsilon)) return false;
      if (!this.sensor_orientation_.epsilonEquals(other.sensor_orientation_, epsilon)) return false;
      if (!this.linear_velocity_.epsilonEquals(other.linear_velocity_, epsilon)) return false;
      if (!this.angular_velocity_.epsilonEquals(other.angular_velocity_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.point_cloud_, other.point_cloud_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DepthCloudMessage)) return false;

      DepthCloudMessage otherMyClass = (DepthCloudMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if (!this.sensor_position_.equals(otherMyClass.sensor_position_)) return false;
      if (!this.sensor_orientation_.equals(otherMyClass.sensor_orientation_)) return false;
      if (!this.linear_velocity_.equals(otherMyClass.linear_velocity_)) return false;
      if (!this.angular_velocity_.equals(otherMyClass.angular_velocity_)) return false;
      if (!this.point_cloud_.equals(otherMyClass.point_cloud_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DepthCloudMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("sensor_position=");
      builder.append(this.sensor_position_);      builder.append(", ");
      builder.append("sensor_orientation=");
      builder.append(this.sensor_orientation_);      builder.append(", ");
      builder.append("linear_velocity=");
      builder.append(this.linear_velocity_);      builder.append(", ");
      builder.append("angular_velocity=");
      builder.append(this.angular_velocity_);      builder.append(", ");
      builder.append("point_cloud=");
      builder.append(this.point_cloud_);
      builder.append("}");
      return builder.toString();
   }
}
