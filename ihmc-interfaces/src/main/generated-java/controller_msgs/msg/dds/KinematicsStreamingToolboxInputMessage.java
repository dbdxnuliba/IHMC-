package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the KinematicsStreamingToolbox API.
       */
public class KinematicsStreamingToolboxInputMessage extends Packet<KinematicsStreamingToolboxInputMessage> implements Settable<KinematicsStreamingToolboxInputMessage>, EpsilonComparable<KinematicsStreamingToolboxInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage head_input_;
   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage left_hand_input_;
   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage right_hand_input_;

   public KinematicsStreamingToolboxInputMessage()
   {
      head_input_ = new controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage();
      left_hand_input_ = new controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage();
      right_hand_input_ = new controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage();
   }

   public KinematicsStreamingToolboxInputMessage(KinematicsStreamingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.staticCopy(other.head_input_, head_input_);
      controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.staticCopy(other.left_hand_input_, left_hand_input_);
      controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.staticCopy(other.right_hand_input_, right_hand_input_);
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


   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage getHeadInput()
   {
      return head_input_;
   }


   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage getLeftHandInput()
   {
      return left_hand_input_;
   }


   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage getRightHandInput()
   {
      return right_hand_input_;
   }


   public static Supplier<KinematicsStreamingToolboxInputMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.head_input_.epsilonEquals(other.head_input_, epsilon)) return false;
      if (!this.left_hand_input_.epsilonEquals(other.left_hand_input_, epsilon)) return false;
      if (!this.right_hand_input_.epsilonEquals(other.right_hand_input_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxInputMessage)) return false;

      KinematicsStreamingToolboxInputMessage otherMyClass = (KinematicsStreamingToolboxInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.head_input_.equals(otherMyClass.head_input_)) return false;
      if (!this.left_hand_input_.equals(otherMyClass.left_hand_input_)) return false;
      if (!this.right_hand_input_.equals(otherMyClass.right_hand_input_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("head_input=");
      builder.append(this.head_input_);      builder.append(", ");
      builder.append("left_hand_input=");
      builder.append(this.left_hand_input_);      builder.append(", ");
      builder.append("right_hand_input=");
      builder.append(this.right_hand_input_);
      builder.append("}");
      return builder.toString();
   }
}
