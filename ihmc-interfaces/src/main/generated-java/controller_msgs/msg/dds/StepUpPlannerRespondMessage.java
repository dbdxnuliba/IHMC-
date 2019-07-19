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

   public StepUpPlannerRespondMessage()
   {
      phases_result_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseResult> (20, new controller_msgs.msg.dds.StepUpPlannerPhaseResultPubSubType());

   }

   public StepUpPlannerRespondMessage(StepUpPlannerRespondMessage other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerRespondMessage other)
   {
      phases_result_.set(other.phases_result_);
   }


   /**
            * The various trajectories are split in every different phase
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseResult>  getPhasesResult()
   {
      return phases_result_;
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

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerRespondMessage {");
      builder.append("phases_result=");
      builder.append(this.phases_result_);
      builder.append("}");
      return builder.toString();
   }
}
