package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the KinematicsStreamingToolbox API.
       */
public class KinematicsStreamingToolboxCalibrationStatusMessage extends Packet<KinematicsStreamingToolboxCalibrationStatusMessage> implements Settable<KinematicsStreamingToolboxCalibrationStatusMessage>, EpsilonComparable<KinematicsStreamingToolboxCalibrationStatusMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public java.lang.StringBuilder current_status_;
   public boolean is_calibration_done_;

   public KinematicsStreamingToolboxCalibrationStatusMessage()
   {
      current_status_ = new java.lang.StringBuilder(255);
   }

   public KinematicsStreamingToolboxCalibrationStatusMessage(KinematicsStreamingToolboxCalibrationStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxCalibrationStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      current_status_.setLength(0);
      current_status_.append(other.current_status_);

      is_calibration_done_ = other.is_calibration_done_;

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

   public void setCurrentStatus(java.lang.String current_status)
   {
      current_status_.setLength(0);
      current_status_.append(current_status);
   }

   public java.lang.String getCurrentStatusAsString()
   {
      return getCurrentStatus().toString();
   }
   public java.lang.StringBuilder getCurrentStatus()
   {
      return current_status_;
   }

   public void setIsCalibrationDone(boolean is_calibration_done)
   {
      is_calibration_done_ = is_calibration_done;
   }
   public boolean getIsCalibrationDone()
   {
      return is_calibration_done_;
   }


   public static Supplier<KinematicsStreamingToolboxCalibrationStatusMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxCalibrationStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxCalibrationStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxCalibrationStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.current_status_, other.current_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_calibration_done_, other.is_calibration_done_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxCalibrationStatusMessage)) return false;

      KinematicsStreamingToolboxCalibrationStatusMessage otherMyClass = (KinematicsStreamingToolboxCalibrationStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.current_status_, otherMyClass.current_status_)) return false;

      if(this.is_calibration_done_ != otherMyClass.is_calibration_done_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxCalibrationStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("current_status=");
      builder.append(this.current_status_);      builder.append(", ");
      builder.append("is_calibration_done=");
      builder.append(this.is_calibration_done_);
      builder.append("}");
      return builder.toString();
   }
}
