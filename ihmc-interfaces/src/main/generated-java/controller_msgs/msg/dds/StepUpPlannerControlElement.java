package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerControlElement extends Packet<StepUpPlannerControlElement> implements Settable<StepUpPlannerControlElement>, EpsilonComparable<StepUpPlannerControlElement>
{
   public double multiplier_;
   public controller_msgs.msg.dds.StepUpPlannerVector2 cop_;

   public StepUpPlannerControlElement()
   {
      cop_ = new controller_msgs.msg.dds.StepUpPlannerVector2();
   }

   public StepUpPlannerControlElement(StepUpPlannerControlElement other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerControlElement other)
   {
      multiplier_ = other.multiplier_;

      controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.staticCopy(other.cop_, cop_);
   }

   public void setMultiplier(double multiplier)
   {
      multiplier_ = multiplier;
   }
   public double getMultiplier()
   {
      return multiplier_;
   }


   public controller_msgs.msg.dds.StepUpPlannerVector2 getCop()
   {
      return cop_;
   }


   public static Supplier<StepUpPlannerControlElementPubSubType> getPubSubType()
   {
      return StepUpPlannerControlElementPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerControlElementPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerControlElement other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.multiplier_, other.multiplier_, epsilon)) return false;

      if (!this.cop_.epsilonEquals(other.cop_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerControlElement)) return false;

      StepUpPlannerControlElement otherMyClass = (StepUpPlannerControlElement) other;

      if(this.multiplier_ != otherMyClass.multiplier_) return false;

      if (!this.cop_.equals(otherMyClass.cop_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerControlElement {");
      builder.append("multiplier=");
      builder.append(this.multiplier_);      builder.append(", ");
      builder.append("cop=");
      builder.append(this.cop_);
      builder.append("}");
      return builder.toString();
   }
}
