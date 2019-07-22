package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerRequestMessage extends Packet<StepUpPlannerRequestMessage> implements Settable<StepUpPlannerRequestMessage>, EpsilonComparable<StepUpPlannerRequestMessage>
{
   /**
            * Pose of each foot and timings. The number should correspond to the one specified in the StepUpPlannerPhaseParameters message
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhase>  phases_;
   /**
            * Position of the CoM to be reached at the end of the final phase
            */
   public us.ihmc.euclid.tuple3D.Vector3D desired_com_position_;
   /**
            * Velocity of the CoM to be reached at the end of the final phase
            */
   public us.ihmc.euclid.tuple3D.Vector3D desired_com_velocity_;
   /**
            * Initial CoM position
            */
   public us.ihmc.euclid.tuple3D.Vector3D initial_com_position_;
   /**
            * Initial CoM velocity
            */
   public us.ihmc.euclid.tuple3D.Vector3D initial_com_velocity_;
   /**
            * Desired value for the final left control
            */
   public controller_msgs.msg.dds.StepUpPlannerControlElement left_desired_final_control_;
   /**
            * Desired value for the final right control
            */
   public controller_msgs.msg.dds.StepUpPlannerControlElement right_desired_final_control_;
   /**
            * Desired leg length
            */
   public double desired_leg_length_ = 1.5;

   public StepUpPlannerRequestMessage()
   {
      phases_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhase> (20, new controller_msgs.msg.dds.StepUpPlannerPhasePubSubType());
      desired_com_position_ = new us.ihmc.euclid.tuple3D.Vector3D();
      desired_com_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      initial_com_position_ = new us.ihmc.euclid.tuple3D.Vector3D();
      initial_com_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      left_desired_final_control_ = new controller_msgs.msg.dds.StepUpPlannerControlElement();
      right_desired_final_control_ = new controller_msgs.msg.dds.StepUpPlannerControlElement();

   }

   public StepUpPlannerRequestMessage(StepUpPlannerRequestMessage other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerRequestMessage other)
   {
      phases_.set(other.phases_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_com_position_, desired_com_position_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_com_velocity_, desired_com_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.initial_com_position_, initial_com_position_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.initial_com_velocity_, initial_com_velocity_);
      controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.staticCopy(other.left_desired_final_control_, left_desired_final_control_);
      controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.staticCopy(other.right_desired_final_control_, right_desired_final_control_);
      desired_leg_length_ = other.desired_leg_length_;

   }


   /**
            * Pose of each foot and timings. The number should correspond to the one specified in the StepUpPlannerPhaseParameters message
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhase>  getPhases()
   {
      return phases_;
   }


   /**
            * Position of the CoM to be reached at the end of the final phase
            */
   public us.ihmc.euclid.tuple3D.Vector3D getDesiredComPosition()
   {
      return desired_com_position_;
   }


   /**
            * Velocity of the CoM to be reached at the end of the final phase
            */
   public us.ihmc.euclid.tuple3D.Vector3D getDesiredComVelocity()
   {
      return desired_com_velocity_;
   }


   /**
            * Initial CoM position
            */
   public us.ihmc.euclid.tuple3D.Vector3D getInitialComPosition()
   {
      return initial_com_position_;
   }


   /**
            * Initial CoM velocity
            */
   public us.ihmc.euclid.tuple3D.Vector3D getInitialComVelocity()
   {
      return initial_com_velocity_;
   }


   /**
            * Desired value for the final left control
            */
   public controller_msgs.msg.dds.StepUpPlannerControlElement getLeftDesiredFinalControl()
   {
      return left_desired_final_control_;
   }


   /**
            * Desired value for the final right control
            */
   public controller_msgs.msg.dds.StepUpPlannerControlElement getRightDesiredFinalControl()
   {
      return right_desired_final_control_;
   }

   /**
            * Desired leg length
            */
   public void setDesiredLegLength(double desired_leg_length)
   {
      desired_leg_length_ = desired_leg_length;
   }
   /**
            * Desired leg length
            */
   public double getDesiredLegLength()
   {
      return desired_leg_length_;
   }


   public static Supplier<StepUpPlannerRequestMessagePubSubType> getPubSubType()
   {
      return StepUpPlannerRequestMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerRequestMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerRequestMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.phases_.size() != other.phases_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.phases_.size(); i++)
         {  if (!this.phases_.get(i).epsilonEquals(other.phases_.get(i), epsilon)) return false; }
      }

      if (!this.desired_com_position_.epsilonEquals(other.desired_com_position_, epsilon)) return false;
      if (!this.desired_com_velocity_.epsilonEquals(other.desired_com_velocity_, epsilon)) return false;
      if (!this.initial_com_position_.epsilonEquals(other.initial_com_position_, epsilon)) return false;
      if (!this.initial_com_velocity_.epsilonEquals(other.initial_com_velocity_, epsilon)) return false;
      if (!this.left_desired_final_control_.epsilonEquals(other.left_desired_final_control_, epsilon)) return false;
      if (!this.right_desired_final_control_.epsilonEquals(other.right_desired_final_control_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_leg_length_, other.desired_leg_length_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerRequestMessage)) return false;

      StepUpPlannerRequestMessage otherMyClass = (StepUpPlannerRequestMessage) other;

      if (!this.phases_.equals(otherMyClass.phases_)) return false;
      if (!this.desired_com_position_.equals(otherMyClass.desired_com_position_)) return false;
      if (!this.desired_com_velocity_.equals(otherMyClass.desired_com_velocity_)) return false;
      if (!this.initial_com_position_.equals(otherMyClass.initial_com_position_)) return false;
      if (!this.initial_com_velocity_.equals(otherMyClass.initial_com_velocity_)) return false;
      if (!this.left_desired_final_control_.equals(otherMyClass.left_desired_final_control_)) return false;
      if (!this.right_desired_final_control_.equals(otherMyClass.right_desired_final_control_)) return false;
      if(this.desired_leg_length_ != otherMyClass.desired_leg_length_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerRequestMessage {");
      builder.append("phases=");
      builder.append(this.phases_);      builder.append(", ");
      builder.append("desired_com_position=");
      builder.append(this.desired_com_position_);      builder.append(", ");
      builder.append("desired_com_velocity=");
      builder.append(this.desired_com_velocity_);      builder.append(", ");
      builder.append("initial_com_position=");
      builder.append(this.initial_com_position_);      builder.append(", ");
      builder.append("initial_com_velocity=");
      builder.append(this.initial_com_velocity_);      builder.append(", ");
      builder.append("left_desired_final_control=");
      builder.append(this.left_desired_final_control_);      builder.append(", ");
      builder.append("right_desired_final_control=");
      builder.append(this.right_desired_final_control_);      builder.append(", ");
      builder.append("desired_leg_length=");
      builder.append(this.desired_leg_length_);
      builder.append("}");
      return builder.toString();
   }
}
