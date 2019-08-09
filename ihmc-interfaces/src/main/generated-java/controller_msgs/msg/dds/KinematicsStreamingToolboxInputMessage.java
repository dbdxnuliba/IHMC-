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
   public boolean control_center_of_mass_;
   public controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage center_of_mass_input_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>  rigid_body_inputs_;

   public KinematicsStreamingToolboxInputMessage()
   {
      center_of_mass_input_ = new controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage();
      rigid_body_inputs_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage> (20, new controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType());

   }

   public KinematicsStreamingToolboxInputMessage(KinematicsStreamingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      control_center_of_mass_ = other.control_center_of_mass_;

      controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.staticCopy(other.center_of_mass_input_, center_of_mass_input_);
      rigid_body_inputs_.set(other.rigid_body_inputs_);
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

   public void setControlCenterOfMass(boolean control_center_of_mass)
   {
      control_center_of_mass_ = control_center_of_mass;
   }
   public boolean getControlCenterOfMass()
   {
      return control_center_of_mass_;
   }


   public controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage getCenterOfMassInput()
   {
      return center_of_mass_input_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>  getRigidBodyInputs()
   {
      return rigid_body_inputs_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.control_center_of_mass_, other.control_center_of_mass_, epsilon)) return false;

      if (!this.center_of_mass_input_.epsilonEquals(other.center_of_mass_input_, epsilon)) return false;
      if (this.rigid_body_inputs_.size() != other.rigid_body_inputs_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.rigid_body_inputs_.size(); i++)
         {  if (!this.rigid_body_inputs_.get(i).epsilonEquals(other.rigid_body_inputs_.get(i), epsilon)) return false; }
      }


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

      if(this.control_center_of_mass_ != otherMyClass.control_center_of_mass_) return false;

      if (!this.center_of_mass_input_.equals(otherMyClass.center_of_mass_input_)) return false;
      if (!this.rigid_body_inputs_.equals(otherMyClass.rigid_body_inputs_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("control_center_of_mass=");
      builder.append(this.control_center_of_mass_);      builder.append(", ");
      builder.append("center_of_mass_input=");
      builder.append(this.center_of_mass_input_);      builder.append(", ");
      builder.append("rigid_body_inputs=");
      builder.append(this.rigid_body_inputs_);
      builder.append("}");
      return builder.toString();
   }
}
