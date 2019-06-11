package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class CollisionManagerMessage extends Packet<CollisionManagerMessage> implements Settable<CollisionManagerMessage>, EpsilonComparable<CollisionManagerMessage>
{
   public float test_;

   public CollisionManagerMessage()
   {
   }

   public CollisionManagerMessage(CollisionManagerMessage other)
   {
      this();
      set(other);
   }

   public void set(CollisionManagerMessage other)
   {
      test_ = other.test_;

   }

   public void setTest(float test)
   {
      test_ = test;
   }
   public float getTest()
   {
      return test_;
   }


   public static Supplier<CollisionManagerMessagePubSubType> getPubSubType()
   {
      return CollisionManagerMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CollisionManagerMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CollisionManagerMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.test_, other.test_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CollisionManagerMessage)) return false;

      CollisionManagerMessage otherMyClass = (CollisionManagerMessage) other;

      if(this.test_ != otherMyClass.test_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CollisionManagerMessage {");
      builder.append("test=");
      builder.append(this.test_);
      builder.append("}");
      return builder.toString();
   }
}
