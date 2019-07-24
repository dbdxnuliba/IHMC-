package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message sends a desired stepping velocity to the step teleop module.
       */
public class QuadrupedTeleopMessage extends Packet<QuadrupedTeleopMessage> implements Settable<QuadrupedTeleopMessage>, EpsilonComparable<QuadrupedTeleopMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Flag to request walking or standing. If true the robot will start or continue to walk with the specified parameters.
            * If false the robot will stop walking and the parameters below are ignored.
            */
   public boolean request_walk_;
   /**
            * Defines if the step list is adjustable
            */
   public boolean are_steps_adjustable_ = true;
   /**
            * Desired planar velocities in m/s and rad/s. X and Y correspond to forward and lateral velocities. Z corresponds to yaw velocity
            */
   public us.ihmc.euclid.tuple3D.Vector3D desired_velocity_;
   /**
            * Desired xgait settings
            */
   public controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket x_gait_settings_;

   public QuadrupedTeleopMessage()
   {
      desired_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      x_gait_settings_ = new controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket();
   }

   public QuadrupedTeleopMessage(QuadrupedTeleopMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedTeleopMessage other)
   {
      sequence_id_ = other.sequence_id_;

      request_walk_ = other.request_walk_;

      are_steps_adjustable_ = other.are_steps_adjustable_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_velocity_, desired_velocity_);
      controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType.staticCopy(other.x_gait_settings_, x_gait_settings_);
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
            * Flag to request walking or standing. If true the robot will start or continue to walk with the specified parameters.
            * If false the robot will stop walking and the parameters below are ignored.
            */
   public void setRequestWalk(boolean request_walk)
   {
      request_walk_ = request_walk;
   }
   /**
            * Flag to request walking or standing. If true the robot will start or continue to walk with the specified parameters.
            * If false the robot will stop walking and the parameters below are ignored.
            */
   public boolean getRequestWalk()
   {
      return request_walk_;
   }

   /**
            * Defines if the step list is adjustable
            */
   public void setAreStepsAdjustable(boolean are_steps_adjustable)
   {
      are_steps_adjustable_ = are_steps_adjustable;
   }
   /**
            * Defines if the step list is adjustable
            */
   public boolean getAreStepsAdjustable()
   {
      return are_steps_adjustable_;
   }


   /**
            * Desired planar velocities in m/s and rad/s. X and Y correspond to forward and lateral velocities. Z corresponds to yaw velocity
            */
   public us.ihmc.euclid.tuple3D.Vector3D getDesiredVelocity()
   {
      return desired_velocity_;
   }


   /**
            * Desired xgait settings
            */
   public controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket getXGaitSettings()
   {
      return x_gait_settings_;
   }


   public static Supplier<QuadrupedTeleopMessagePubSubType> getPubSubType()
   {
      return QuadrupedTeleopMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedTeleopMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTeleopMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_walk_, other.request_walk_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.are_steps_adjustable_, other.are_steps_adjustable_, epsilon)) return false;

      if (!this.desired_velocity_.epsilonEquals(other.desired_velocity_, epsilon)) return false;
      if (!this.x_gait_settings_.epsilonEquals(other.x_gait_settings_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedTeleopMessage)) return false;

      QuadrupedTeleopMessage otherMyClass = (QuadrupedTeleopMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.request_walk_ != otherMyClass.request_walk_) return false;

      if(this.are_steps_adjustable_ != otherMyClass.are_steps_adjustable_) return false;

      if (!this.desired_velocity_.equals(otherMyClass.desired_velocity_)) return false;
      if (!this.x_gait_settings_.equals(otherMyClass.x_gait_settings_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedTeleopMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("request_walk=");
      builder.append(this.request_walk_);      builder.append(", ");
      builder.append("are_steps_adjustable=");
      builder.append(this.are_steps_adjustable_);      builder.append(", ");
      builder.append("desired_velocity=");
      builder.append(this.desired_velocity_);      builder.append(", ");
      builder.append("x_gait_settings=");
      builder.append(this.x_gait_settings_);
      builder.append("}");
      return builder.toString();
   }
}
