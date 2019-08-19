package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the KinematicsStreamingToolbox API.
       * This message can be used to configure the toolbox.
       */
public class KinematicsStreamingToolboxConfigurationMessage extends Packet<KinematicsStreamingToolboxConfigurationMessage> implements Settable<KinematicsStreamingToolboxConfigurationMessage>, EpsilonComparable<KinematicsStreamingToolboxConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Request a re-calibration. If the toolbox is currently streaming or validating, then it stops and falls back to the calibration state.
            */
   public boolean request_calibration_;

   public KinematicsStreamingToolboxConfigurationMessage()
   {
   }

   public KinematicsStreamingToolboxConfigurationMessage(KinematicsStreamingToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      request_calibration_ = other.request_calibration_;

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

   /**
            * Request a re-calibration. If the toolbox is currently streaming or validating, then it stops and falls back to the calibration state.
            */
   public void setRequestCalibration(boolean request_calibration)
   {
      request_calibration_ = request_calibration;
   }
   /**
            * Request a re-calibration. If the toolbox is currently streaming or validating, then it stops and falls back to the calibration state.
            */
   public boolean getRequestCalibration()
   {
      return request_calibration_;
   }


   public static Supplier<KinematicsStreamingToolboxConfigurationMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_calibration_, other.request_calibration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxConfigurationMessage)) return false;

      KinematicsStreamingToolboxConfigurationMessage otherMyClass = (KinematicsStreamingToolboxConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.request_calibration_ != otherMyClass.request_calibration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("request_calibration=");
      builder.append(this.request_calibration_);
      builder.append("}");
      return builder.toString();
   }
}
