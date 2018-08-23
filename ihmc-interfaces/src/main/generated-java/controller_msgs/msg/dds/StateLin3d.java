package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This contains the 3D representation of a linear state, including:
       * position, velocity, acceleration
       */
public class StateLin3d extends Packet<StateLin3d> implements Settable<StateLin3d>, EpsilonComparable<StateLin3d>
{
   public us.ihmc.euclid.tuple3D.Point3D pos_;
   public us.ihmc.euclid.tuple3D.Vector3D vel_;
   public us.ihmc.euclid.tuple3D.Vector3D acc_;

   public StateLin3d()
   {
      pos_ = new us.ihmc.euclid.tuple3D.Point3D();
      vel_ = new us.ihmc.euclid.tuple3D.Vector3D();
      acc_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public StateLin3d(StateLin3d other)
   {
      this();
      set(other);
   }

   public void set(StateLin3d other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.pos_, pos_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.vel_, vel_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.acc_, acc_);
   }


   public us.ihmc.euclid.tuple3D.Point3D getPos()
   {
      return pos_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getVel()
   {
      return vel_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getAcc()
   {
      return acc_;
   }


   public static Supplier<StateLin3dPubSubType> getPubSubType()
   {
      return StateLin3dPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StateLin3dPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StateLin3d other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.pos_.epsilonEquals(other.pos_, epsilon)) return false;
      if (!this.vel_.epsilonEquals(other.vel_, epsilon)) return false;
      if (!this.acc_.epsilonEquals(other.acc_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StateLin3d)) return false;

      StateLin3d otherMyClass = (StateLin3d) other;

      if (!this.pos_.equals(otherMyClass.pos_)) return false;
      if (!this.vel_.equals(otherMyClass.vel_)) return false;
      if (!this.acc_.equals(otherMyClass.acc_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StateLin3d {");
      builder.append("pos=");
      builder.append(this.pos_);      builder.append(", ");
      builder.append("vel=");
      builder.append(this.vel_);      builder.append(", ");
      builder.append("acc=");
      builder.append(this.acc_);
      builder.append("}");
      return builder.toString();
   }
}
