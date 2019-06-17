package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class CollisionManagerMessage extends Packet<CollisionManagerMessage> implements Settable<CollisionManagerMessage>, EpsilonComparable<CollisionManagerMessage>
{
   /**
            * List of the planar regions considered for collision avoidance.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage>  planar_regions_list_;

   public CollisionManagerMessage()
   {
      planar_regions_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage> (100, new controller_msgs.msg.dds.PlanarRegionMessagePubSubType());

   }

   public CollisionManagerMessage(CollisionManagerMessage other)
   {
      this();
      set(other);
   }

   public void set(CollisionManagerMessage other)
   {
      planar_regions_list_.set(other.planar_regions_list_);
   }


   /**
            * List of the planar regions considered for collision avoidance.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage>  getPlanarRegionsList()
   {
      return planar_regions_list_;
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

      if (this.planar_regions_list_.size() != other.planar_regions_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.planar_regions_list_.size(); i++)
         {  if (!this.planar_regions_list_.get(i).epsilonEquals(other.planar_regions_list_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CollisionManagerMessage)) return false;

      CollisionManagerMessage otherMyClass = (CollisionManagerMessage) other;

      if (!this.planar_regions_list_.equals(otherMyClass.planar_regions_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CollisionManagerMessage {");
      builder.append("planar_regions_list=");
      builder.append(this.planar_regions_list_);
      builder.append("}");
      return builder.toString();
   }
}
