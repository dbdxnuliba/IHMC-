package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       */
public class KinematicsStreamingToolboxStatusMessage extends Packet<KinematicsStreamingToolboxStatusMessage> implements Settable<KinematicsStreamingToolboxStatusMessage>, EpsilonComparable<KinematicsStreamingToolboxStatusMessage>
{
   public static final byte KSTSTATE_SLEEP = (byte) 0;
   public static final byte KSTSTATE_CALIBRATION = (byte) 1;
   public static final byte KSTSTATE_VALIDATION = (byte) 2;
   public static final byte KSTSTATE_STREAMING = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Indicates the current state of the toolbox:
            */
   public byte current_state_ = (byte) 255;

   public KinematicsStreamingToolboxStatusMessage()
   {
   }

   public KinematicsStreamingToolboxStatusMessage(KinematicsStreamingToolboxStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      current_state_ = other.current_state_;

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
            * Indicates the current state of the toolbox:
            */
   public void setCurrentState(byte current_state)
   {
      current_state_ = current_state;
   }
   /**
            * Indicates the current state of the toolbox:
            */
   public byte getCurrentState()
   {
      return current_state_;
   }


   public static Supplier<KinematicsStreamingToolboxStatusMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_state_, other.current_state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxStatusMessage)) return false;

      KinematicsStreamingToolboxStatusMessage otherMyClass = (KinematicsStreamingToolboxStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.current_state_ != otherMyClass.current_state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("current_state=");
      builder.append(this.current_state_);
      builder.append("}");
      return builder.toString();
   }
}
