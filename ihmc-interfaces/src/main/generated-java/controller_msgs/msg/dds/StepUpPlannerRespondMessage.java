package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerRespondMessage extends Packet<StepUpPlannerRespondMessage> implements Settable<StepUpPlannerRespondMessage>, EpsilonComparable<StepUpPlannerRespondMessage>
{
   /**
            * The various trajectories are split in every different phase
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseResult>  phases_result_;
   /**
            * The total duration of the generated trajectories
            */
   public double total_duration_;
   /**
            * A collection of messages containing the CoM trajectory (0 elements in case not required in the parameters message)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CenterOfMassTrajectoryMessage>  com_messages_;
   /**
            * A collection of messages containing the pelvis height trajectory (0 elements in case not required in the parameters message)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PelvisHeightTrajectoryMessage>  pelvis_height_messages_;
   /**
            * A collection of messages containing the footsteps (0 elements in case not required in the parameters message)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataListMessage>  foostep_messages_;

   public StepUpPlannerRespondMessage()
   {
      phases_result_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseResult> (20, new controller_msgs.msg.dds.StepUpPlannerPhaseResultPubSubType());
      com_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CenterOfMassTrajectoryMessage> (20, new controller_msgs.msg.dds.CenterOfMassTrajectoryMessagePubSubType());
      pelvis_height_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PelvisHeightTrajectoryMessage> (20, new controller_msgs.msg.dds.PelvisHeightTrajectoryMessagePubSubType());
      foostep_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataListMessage> (20, new controller_msgs.msg.dds.FootstepDataListMessagePubSubType());

   }

   public StepUpPlannerRespondMessage(StepUpPlannerRespondMessage other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerRespondMessage other)
   {
      phases_result_.set(other.phases_result_);
      total_duration_ = other.total_duration_;

      com_messages_.set(other.com_messages_);
      pelvis_height_messages_.set(other.pelvis_height_messages_);
      foostep_messages_.set(other.foostep_messages_);
   }


   /**
            * The various trajectories are split in every different phase
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseResult>  getPhasesResult()
   {
      return phases_result_;
   }

   /**
            * The total duration of the generated trajectories
            */
   public void setTotalDuration(double total_duration)
   {
      total_duration_ = total_duration;
   }
   /**
            * The total duration of the generated trajectories
            */
   public double getTotalDuration()
   {
      return total_duration_;
   }


   /**
            * A collection of messages containing the CoM trajectory (0 elements in case not required in the parameters message)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CenterOfMassTrajectoryMessage>  getComMessages()
   {
      return com_messages_;
   }


   /**
            * A collection of messages containing the pelvis height trajectory (0 elements in case not required in the parameters message)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PelvisHeightTrajectoryMessage>  getPelvisHeightMessages()
   {
      return pelvis_height_messages_;
   }


   /**
            * A collection of messages containing the footsteps (0 elements in case not required in the parameters message)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataListMessage>  getFoostepMessages()
   {
      return foostep_messages_;
   }


   public static Supplier<StepUpPlannerRespondMessagePubSubType> getPubSubType()
   {
      return StepUpPlannerRespondMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerRespondMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerRespondMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.phases_result_.size() != other.phases_result_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.phases_result_.size(); i++)
         {  if (!this.phases_result_.get(i).epsilonEquals(other.phases_result_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_duration_, other.total_duration_, epsilon)) return false;

      if (this.com_messages_.size() != other.com_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.com_messages_.size(); i++)
         {  if (!this.com_messages_.get(i).epsilonEquals(other.com_messages_.get(i), epsilon)) return false; }
      }

      if (this.pelvis_height_messages_.size() != other.pelvis_height_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.pelvis_height_messages_.size(); i++)
         {  if (!this.pelvis_height_messages_.get(i).epsilonEquals(other.pelvis_height_messages_.get(i), epsilon)) return false; }
      }

      if (this.foostep_messages_.size() != other.foostep_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.foostep_messages_.size(); i++)
         {  if (!this.foostep_messages_.get(i).epsilonEquals(other.foostep_messages_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerRespondMessage)) return false;

      StepUpPlannerRespondMessage otherMyClass = (StepUpPlannerRespondMessage) other;

      if (!this.phases_result_.equals(otherMyClass.phases_result_)) return false;
      if(this.total_duration_ != otherMyClass.total_duration_) return false;

      if (!this.com_messages_.equals(otherMyClass.com_messages_)) return false;
      if (!this.pelvis_height_messages_.equals(otherMyClass.pelvis_height_messages_)) return false;
      if (!this.foostep_messages_.equals(otherMyClass.foostep_messages_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerRespondMessage {");
      builder.append("phases_result=");
      builder.append(this.phases_result_);      builder.append(", ");
      builder.append("total_duration=");
      builder.append(this.total_duration_);      builder.append(", ");
      builder.append("com_messages=");
      builder.append(this.com_messages_);      builder.append(", ");
      builder.append("pelvis_height_messages=");
      builder.append(this.pelvis_height_messages_);      builder.append(", ");
      builder.append("foostep_messages=");
      builder.append(this.foostep_messages_);
      builder.append("}");
      return builder.toString();
   }
}
