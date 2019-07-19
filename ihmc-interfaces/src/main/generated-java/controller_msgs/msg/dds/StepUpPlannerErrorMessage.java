package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerErrorMessage extends Packet<StepUpPlannerErrorMessage> implements Settable<StepUpPlannerErrorMessage>, EpsilonComparable<StepUpPlannerErrorMessage>
{
   /**
            * Description of the error
            */
   public java.lang.StringBuilder error_description_;
   /**
            * (0) No Error. Using this message to ack a parameter message
            * (1) Parameters error
            * (2) Parameters not set
            * (3) Request error
            * (4) Error in the solver
            */
   public byte error_code_;
   /**
            * Send back the ID of the received parameters message
            */
   public long sequence_id_received_;

   public StepUpPlannerErrorMessage()
   {
      error_description_ = new java.lang.StringBuilder(255);
   }

   public StepUpPlannerErrorMessage(StepUpPlannerErrorMessage other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerErrorMessage other)
   {
      error_description_.setLength(0);
      error_description_.append(other.error_description_);

      error_code_ = other.error_code_;

      sequence_id_received_ = other.sequence_id_received_;

   }

   /**
            * Description of the error
            */
   public void setErrorDescription(java.lang.String error_description)
   {
      error_description_.setLength(0);
      error_description_.append(error_description);
   }

   /**
            * Description of the error
            */
   public java.lang.String getErrorDescriptionAsString()
   {
      return getErrorDescription().toString();
   }
   /**
            * Description of the error
            */
   public java.lang.StringBuilder getErrorDescription()
   {
      return error_description_;
   }

   /**
            * (0) No Error. Using this message to ack a parameter message
            * (1) Parameters error
            * (2) Parameters not set
            * (3) Request error
            * (4) Error in the solver
            */
   public void setErrorCode(byte error_code)
   {
      error_code_ = error_code;
   }
   /**
            * (0) No Error. Using this message to ack a parameter message
            * (1) Parameters error
            * (2) Parameters not set
            * (3) Request error
            * (4) Error in the solver
            */
   public byte getErrorCode()
   {
      return error_code_;
   }

   /**
            * Send back the ID of the received parameters message
            */
   public void setSequenceIdReceived(long sequence_id_received)
   {
      sequence_id_received_ = sequence_id_received;
   }
   /**
            * Send back the ID of the received parameters message
            */
   public long getSequenceIdReceived()
   {
      return sequence_id_received_;
   }


   public static Supplier<StepUpPlannerErrorMessagePubSubType> getPubSubType()
   {
      return StepUpPlannerErrorMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerErrorMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerErrorMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.error_description_, other.error_description_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_code_, other.error_code_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_received_, other.sequence_id_received_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerErrorMessage)) return false;

      StepUpPlannerErrorMessage otherMyClass = (StepUpPlannerErrorMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.error_description_, otherMyClass.error_description_)) return false;

      if(this.error_code_ != otherMyClass.error_code_) return false;

      if(this.sequence_id_received_ != otherMyClass.sequence_id_received_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerErrorMessage {");
      builder.append("error_description=");
      builder.append(this.error_description_);      builder.append(", ");
      builder.append("error_code=");
      builder.append(this.error_code_);      builder.append(", ");
      builder.append("sequence_id_received=");
      builder.append(this.sequence_id_received_);
      builder.append("}");
      return builder.toString();
   }
}
