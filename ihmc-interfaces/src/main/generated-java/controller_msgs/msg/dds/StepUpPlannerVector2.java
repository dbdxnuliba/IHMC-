package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerVector2 extends Packet<StepUpPlannerVector2> implements Settable<StepUpPlannerVector2>, EpsilonComparable<StepUpPlannerVector2>
{
   public double x_;
   public double y_;

   public StepUpPlannerVector2()
   {
   }

   public StepUpPlannerVector2(StepUpPlannerVector2 other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerVector2 other)
   {
      x_ = other.x_;

      y_ = other.y_;

   }

   public void setX(double x)
   {
      x_ = x;
   }
   public double getX()
   {
      return x_;
   }

   public void setY(double y)
   {
      y_ = y;
   }
   public double getY()
   {
      return y_;
   }


   public static Supplier<StepUpPlannerVector2PubSubType> getPubSubType()
   {
      return StepUpPlannerVector2PubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerVector2PubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerVector2 other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_, other.x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_, other.y_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerVector2)) return false;

      StepUpPlannerVector2 otherMyClass = (StepUpPlannerVector2) other;

      if(this.x_ != otherMyClass.x_) return false;

      if(this.y_ != otherMyClass.y_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerVector2 {");
      builder.append("x=");
      builder.append(this.x_);      builder.append(", ");
      builder.append("y=");
      builder.append(this.y_);
      builder.append("}");
      return builder.toString();
   }
}
