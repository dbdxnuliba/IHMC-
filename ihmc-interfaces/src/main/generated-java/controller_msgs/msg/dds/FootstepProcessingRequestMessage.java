package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep processing toolbox API
       */
public class FootstepProcessingRequestMessage extends Packet<FootstepProcessingRequestMessage> implements Settable<FootstepProcessingRequestMessage>, EpsilonComparable<FootstepProcessingRequestMessage>
{
   /**
            * Footstep list containing steps to be mutated
            */
   public controller_msgs.msg.dds.FootstepDataListMessage footstep_data_list_message_;
   /**
            * Specifies the position of the initial left stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D left_foot_location_;
   /**
            * Specifies the orientation of the initial left stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion left_foot_orientation_;
   /**
            * Specifies the position of the initial right stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D right_foot_location_;
   /**
            * Specifies the orientation of the initial right stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion right_foot_orientation_;

   public FootstepProcessingRequestMessage()
   {
      footstep_data_list_message_ = new controller_msgs.msg.dds.FootstepDataListMessage();
      left_foot_location_ = new us.ihmc.euclid.tuple3D.Point3D();
      left_foot_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      right_foot_location_ = new us.ihmc.euclid.tuple3D.Point3D();
      right_foot_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public FootstepProcessingRequestMessage(FootstepProcessingRequestMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepProcessingRequestMessage other)
   {
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footstep_data_list_message_, footstep_data_list_message_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.left_foot_location_, left_foot_location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.left_foot_orientation_, left_foot_orientation_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.right_foot_location_, right_foot_location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.right_foot_orientation_, right_foot_orientation_);
   }


   /**
            * Footstep list containing steps to be mutated
            */
   public controller_msgs.msg.dds.FootstepDataListMessage getFootstepDataListMessage()
   {
      return footstep_data_list_message_;
   }


   /**
            * Specifies the position of the initial left stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getLeftFootLocation()
   {
      return left_foot_location_;
   }


   /**
            * Specifies the orientation of the initial left stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getLeftFootOrientation()
   {
      return left_foot_orientation_;
   }


   /**
            * Specifies the position of the initial right stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getRightFootLocation()
   {
      return right_foot_location_;
   }


   /**
            * Specifies the orientation of the initial right stance foot (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getRightFootOrientation()
   {
      return right_foot_orientation_;
   }


   public static Supplier<FootstepProcessingRequestMessagePubSubType> getPubSubType()
   {
      return FootstepProcessingRequestMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepProcessingRequestMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepProcessingRequestMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.footstep_data_list_message_.epsilonEquals(other.footstep_data_list_message_, epsilon)) return false;
      if (!this.left_foot_location_.epsilonEquals(other.left_foot_location_, epsilon)) return false;
      if (!this.left_foot_orientation_.epsilonEquals(other.left_foot_orientation_, epsilon)) return false;
      if (!this.right_foot_location_.epsilonEquals(other.right_foot_location_, epsilon)) return false;
      if (!this.right_foot_orientation_.epsilonEquals(other.right_foot_orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepProcessingRequestMessage)) return false;

      FootstepProcessingRequestMessage otherMyClass = (FootstepProcessingRequestMessage) other;

      if (!this.footstep_data_list_message_.equals(otherMyClass.footstep_data_list_message_)) return false;
      if (!this.left_foot_location_.equals(otherMyClass.left_foot_location_)) return false;
      if (!this.left_foot_orientation_.equals(otherMyClass.left_foot_orientation_)) return false;
      if (!this.right_foot_location_.equals(otherMyClass.right_foot_location_)) return false;
      if (!this.right_foot_orientation_.equals(otherMyClass.right_foot_orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepProcessingRequestMessage {");
      builder.append("footstep_data_list_message=");
      builder.append(this.footstep_data_list_message_);      builder.append(", ");
      builder.append("left_foot_location=");
      builder.append(this.left_foot_location_);      builder.append(", ");
      builder.append("left_foot_orientation=");
      builder.append(this.left_foot_orientation_);      builder.append(", ");
      builder.append("right_foot_location=");
      builder.append(this.right_foot_location_);      builder.append(", ");
      builder.append("right_foot_orientation=");
      builder.append(this.right_foot_orientation_);
      builder.append("}");
      return builder.toString();
   }
}
