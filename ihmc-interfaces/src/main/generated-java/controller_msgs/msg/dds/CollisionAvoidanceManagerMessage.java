package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class CollisionAvoidanceManagerMessage extends Packet<CollisionAvoidanceManagerMessage> implements Settable<CollisionAvoidanceManagerMessage>, EpsilonComparable<CollisionAvoidanceManagerMessage>
{
   public static final byte MODE_OVERRIDE = (byte) 0;
   public static final byte MODE_ADD = (byte) 1;
   /**
            * List of the planar regions considered for collision avoidance.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage>  planar_regions_list_;
   /**
            * Considers only the edges for collision avoidance
            */
   public boolean consider_only_edges_;
   /**
            * When equal to MODE_OVERRIDE, regions previously sent are cleared and only these regions will be considered.
            * When equal to MODE_ADD, the regions sent with this message are added. The "consider_only_edges" parameter will affect only the regions sent within this message
            */
   public byte mode_;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public CollisionAvoidanceManagerMessage()
   {
      planar_regions_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage> (20, new controller_msgs.msg.dds.PlanarRegionMessagePubSubType());

   }

   public CollisionAvoidanceManagerMessage(CollisionAvoidanceManagerMessage other)
   {
      this();
      set(other);
   }

   public void set(CollisionAvoidanceManagerMessage other)
   {
      planar_regions_list_.set(other.planar_regions_list_);
      consider_only_edges_ = other.consider_only_edges_;

      mode_ = other.mode_;

      sequence_id_ = other.sequence_id_;

   }


   /**
            * List of the planar regions considered for collision avoidance.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage>  getPlanarRegionsList()
   {
      return planar_regions_list_;
   }

   /**
            * Considers only the edges for collision avoidance
            */
   public void setConsiderOnlyEdges(boolean consider_only_edges)
   {
      consider_only_edges_ = consider_only_edges;
   }
   /**
            * Considers only the edges for collision avoidance
            */
   public boolean getConsiderOnlyEdges()
   {
      return consider_only_edges_;
   }

   /**
            * When equal to MODE_OVERRIDE, regions previously sent are cleared and only these regions will be considered.
            * When equal to MODE_ADD, the regions sent with this message are added. The "consider_only_edges" parameter will affect only the regions sent within this message
            */
   public void setMode(byte mode)
   {
      mode_ = mode;
   }
   /**
            * When equal to MODE_OVERRIDE, regions previously sent are cleared and only these regions will be considered.
            * When equal to MODE_ADD, the regions sent with this message are added. The "consider_only_edges" parameter will affect only the regions sent within this message
            */
   public byte getMode()
   {
      return mode_;
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


   public static Supplier<CollisionAvoidanceManagerMessagePubSubType> getPubSubType()
   {
      return CollisionAvoidanceManagerMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CollisionAvoidanceManagerMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CollisionAvoidanceManagerMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.planar_regions_list_.size() != other.planar_regions_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.planar_regions_list_.size(); i++)
         {  if (!this.planar_regions_list_.get(i).epsilonEquals(other.planar_regions_list_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.consider_only_edges_, other.consider_only_edges_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_, other.mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CollisionAvoidanceManagerMessage)) return false;

      CollisionAvoidanceManagerMessage otherMyClass = (CollisionAvoidanceManagerMessage) other;

      if (!this.planar_regions_list_.equals(otherMyClass.planar_regions_list_)) return false;
      if(this.consider_only_edges_ != otherMyClass.consider_only_edges_) return false;

      if(this.mode_ != otherMyClass.mode_) return false;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CollisionAvoidanceManagerMessage {");
      builder.append("planar_regions_list=");
      builder.append(this.planar_regions_list_);      builder.append(", ");
      builder.append("consider_only_edges=");
      builder.append(this.consider_only_edges_);      builder.append(", ");
      builder.append("mode=");
      builder.append(this.mode_);      builder.append(", ");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
