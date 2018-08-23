package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The state of the 6D base of a system
       */
public class State6d extends Packet<State6d> implements Settable<State6d>, EpsilonComparable<State6d>
{
   /**
            * The 6D linear and angular position, orientation maps base to world
            */
   public us.ihmc.euclid.geometry.Pose3D pose_;
   /**
            * The 6D linear and angular velocity
            */
   public geometry_msgs.msg.dds.Twist twist_;
   /**
            * The 6D linear and angular acceleration
            */
   public geometry_msgs.msg.dds.Accel accel_;

   public State6d()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
      twist_ = new geometry_msgs.msg.dds.Twist();
      accel_ = new geometry_msgs.msg.dds.Accel();
   }

   public State6d(State6d other)
   {
      this();
      set(other);
   }

   public void set(State6d other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      geometry_msgs.msg.dds.TwistPubSubType.staticCopy(other.twist_, twist_);
      geometry_msgs.msg.dds.AccelPubSubType.staticCopy(other.accel_, accel_);
   }


   /**
            * The 6D linear and angular position, orientation maps base to world
            */
   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   /**
            * The 6D linear and angular velocity
            */
   public geometry_msgs.msg.dds.Twist getTwist()
   {
      return twist_;
   }


   /**
            * The 6D linear and angular acceleration
            */
   public geometry_msgs.msg.dds.Accel getAccel()
   {
      return accel_;
   }


   public static Supplier<State6dPubSubType> getPubSubType()
   {
      return State6dPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return State6dPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(State6d other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;
      if (!this.twist_.epsilonEquals(other.twist_, epsilon)) return false;
      if (!this.accel_.epsilonEquals(other.accel_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof State6d)) return false;

      State6d otherMyClass = (State6d) other;

      if (!this.pose_.equals(otherMyClass.pose_)) return false;
      if (!this.twist_.equals(otherMyClass.twist_)) return false;
      if (!this.accel_.equals(otherMyClass.accel_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("State6d {");
      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");
      builder.append("twist=");
      builder.append(this.twist_);      builder.append(", ");
      builder.append("accel=");
      builder.append(this.accel_);
      builder.append("}");
      return builder.toString();
   }
}
