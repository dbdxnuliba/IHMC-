package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message containing the result of the Step-Up planner for a single phase. The dT is equal to the duration divided by the number of element in the vectors.
       */
public class StepUpPlannerPhaseResult extends Packet<StepUpPlannerPhaseResult> implements Settable<StepUpPlannerPhaseResult>, EpsilonComparable<StepUpPlannerPhaseResult>
{
   /**
            * CoM position trajectory of this phase
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  com_position_;
   /**
            * CoM velocity trajectory of this phase
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  com_velocity_;
   /**
            * CoM acceleration trajectory of this phase
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  com_acceleration_;
   /**
            * Left control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerControlElement>  left_controls_;
   /**
            * Right control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerControlElement>  right_controls_;
   /**
            * Duration of the phase
            */
   public double duration_;

   public StepUpPlannerPhaseResult()
   {
      com_position_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (200, new geometry_msgs.msg.dds.Vector3PubSubType());
      com_velocity_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (200, new geometry_msgs.msg.dds.Vector3PubSubType());
      com_acceleration_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (200, new geometry_msgs.msg.dds.Vector3PubSubType());
      left_controls_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerControlElement> (200, new controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType());
      right_controls_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerControlElement> (200, new controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType());

   }

   public StepUpPlannerPhaseResult(StepUpPlannerPhaseResult other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerPhaseResult other)
   {
      com_position_.set(other.com_position_);
      com_velocity_.set(other.com_velocity_);
      com_acceleration_.set(other.com_acceleration_);
      left_controls_.set(other.left_controls_);
      right_controls_.set(other.right_controls_);
      duration_ = other.duration_;

   }


   /**
            * CoM position trajectory of this phase
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getComPosition()
   {
      return com_position_;
   }


   /**
            * CoM velocity trajectory of this phase
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getComVelocity()
   {
      return com_velocity_;
   }


   /**
            * CoM acceleration trajectory of this phase
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getComAcceleration()
   {
      return com_acceleration_;
   }


   /**
            * Left control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerControlElement>  getLeftControls()
   {
      return left_controls_;
   }


   /**
            * Right control values (the first element is the multiplier, the other two are the x and y coordinate of the CoP in foot frame)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerControlElement>  getRightControls()
   {
      return right_controls_;
   }

   /**
            * Duration of the phase
            */
   public void setDuration(double duration)
   {
      duration_ = duration;
   }
   /**
            * Duration of the phase
            */
   public double getDuration()
   {
      return duration_;
   }


   public static Supplier<StepUpPlannerPhaseResultPubSubType> getPubSubType()
   {
      return StepUpPlannerPhaseResultPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerPhaseResultPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerPhaseResult other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.com_position_.size() != other.com_position_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.com_position_.size(); i++)
         {  if (!this.com_position_.get(i).epsilonEquals(other.com_position_.get(i), epsilon)) return false; }
      }

      if (this.com_velocity_.size() != other.com_velocity_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.com_velocity_.size(); i++)
         {  if (!this.com_velocity_.get(i).epsilonEquals(other.com_velocity_.get(i), epsilon)) return false; }
      }

      if (this.com_acceleration_.size() != other.com_acceleration_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.com_acceleration_.size(); i++)
         {  if (!this.com_acceleration_.get(i).epsilonEquals(other.com_acceleration_.get(i), epsilon)) return false; }
      }

      if (this.left_controls_.size() != other.left_controls_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.left_controls_.size(); i++)
         {  if (!this.left_controls_.get(i).epsilonEquals(other.left_controls_.get(i), epsilon)) return false; }
      }

      if (this.right_controls_.size() != other.right_controls_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.right_controls_.size(); i++)
         {  if (!this.right_controls_.get(i).epsilonEquals(other.right_controls_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.duration_, other.duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerPhaseResult)) return false;

      StepUpPlannerPhaseResult otherMyClass = (StepUpPlannerPhaseResult) other;

      if (!this.com_position_.equals(otherMyClass.com_position_)) return false;
      if (!this.com_velocity_.equals(otherMyClass.com_velocity_)) return false;
      if (!this.com_acceleration_.equals(otherMyClass.com_acceleration_)) return false;
      if (!this.left_controls_.equals(otherMyClass.left_controls_)) return false;
      if (!this.right_controls_.equals(otherMyClass.right_controls_)) return false;
      if(this.duration_ != otherMyClass.duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerPhaseResult {");
      builder.append("com_position=");
      builder.append(this.com_position_);      builder.append(", ");
      builder.append("com_velocity=");
      builder.append(this.com_velocity_);      builder.append(", ");
      builder.append("com_acceleration=");
      builder.append(this.com_acceleration_);      builder.append(", ");
      builder.append("left_controls=");
      builder.append(this.left_controls_);      builder.append(", ");
      builder.append("right_controls=");
      builder.append(this.right_controls_);      builder.append(", ");
      builder.append("duration=");
      builder.append(this.duration_);
      builder.append("}");
      return builder.toString();
   }
}
