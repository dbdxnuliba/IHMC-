package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class RobotStateCartesian extends Packet<RobotStateCartesian> implements Settable<RobotStateCartesian>, EpsilonComparable<RobotStateCartesian>
{
   /**
            * The state of a robot expressed in the Cartesian frame
            * global time along trajectory
            */
   public controller_msgs.msg.dds.Duration time_from_start_;
   /**
            * Position, velocity and acceleration of the base expressed in world frame
            * The orientation quaternion maps base to world frame.
            * base pos/vel/acc in world
            */
   public controller_msgs.msg.dds.State6d base_;
   /**
            * endeffector pos/vel/acc in world
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StateLin3d>  ee_motion_;
   /**
            * endeffector forces expressed in world
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  ee_forces_;
   /**
            * True if the foot is touching the environment
            */
   public us.ihmc.idl.IDLSequence.Boolean  ee_contact_;

   public RobotStateCartesian()
   {
      time_from_start_ = new controller_msgs.msg.dds.Duration();
      base_ = new controller_msgs.msg.dds.State6d();
      ee_motion_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StateLin3d> (100, new controller_msgs.msg.dds.StateLin3dPubSubType());
      ee_forces_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (100, new geometry_msgs.msg.dds.Vector3PubSubType());
      ee_contact_ = new us.ihmc.idl.IDLSequence.Boolean (100, "type_7");


   }

   public RobotStateCartesian(RobotStateCartesian other)
   {
      this();
      set(other);
   }

   public void set(RobotStateCartesian other)
   {
      controller_msgs.msg.dds.DurationPubSubType.staticCopy(other.time_from_start_, time_from_start_);
      controller_msgs.msg.dds.State6dPubSubType.staticCopy(other.base_, base_);
      ee_motion_.set(other.ee_motion_);
      ee_forces_.set(other.ee_forces_);
      ee_contact_.set(other.ee_contact_);
   }


   /**
            * The state of a robot expressed in the Cartesian frame
            * global time along trajectory
            */
   public controller_msgs.msg.dds.Duration getTimeFromStart()
   {
      return time_from_start_;
   }


   /**
            * Position, velocity and acceleration of the base expressed in world frame
            * The orientation quaternion maps base to world frame.
            * base pos/vel/acc in world
            */
   public controller_msgs.msg.dds.State6d getBase()
   {
      return base_;
   }


   /**
            * endeffector pos/vel/acc in world
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StateLin3d>  getEeMotion()
   {
      return ee_motion_;
   }


   /**
            * endeffector forces expressed in world
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getEeForces()
   {
      return ee_forces_;
   }


   /**
            * True if the foot is touching the environment
            */
   public us.ihmc.idl.IDLSequence.Boolean  getEeContact()
   {
      return ee_contact_;
   }


   public static Supplier<RobotStateCartesianPubSubType> getPubSubType()
   {
      return RobotStateCartesianPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RobotStateCartesianPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RobotStateCartesian other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.time_from_start_.epsilonEquals(other.time_from_start_, epsilon)) return false;
      if (!this.base_.epsilonEquals(other.base_, epsilon)) return false;
      if (this.ee_motion_.size() != other.ee_motion_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.ee_motion_.size(); i++)
         {  if (!this.ee_motion_.get(i).epsilonEquals(other.ee_motion_.get(i), epsilon)) return false; }
      }

      if (this.ee_forces_.size() != other.ee_forces_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.ee_forces_.size(); i++)
         {  if (!this.ee_forces_.get(i).epsilonEquals(other.ee_forces_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBooleanSequence(this.ee_contact_, other.ee_contact_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RobotStateCartesian)) return false;

      RobotStateCartesian otherMyClass = (RobotStateCartesian) other;

      if (!this.time_from_start_.equals(otherMyClass.time_from_start_)) return false;
      if (!this.base_.equals(otherMyClass.base_)) return false;
      if (!this.ee_motion_.equals(otherMyClass.ee_motion_)) return false;
      if (!this.ee_forces_.equals(otherMyClass.ee_forces_)) return false;
      if (!this.ee_contact_.equals(otherMyClass.ee_contact_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotStateCartesian {");
      builder.append("time_from_start=");
      builder.append(this.time_from_start_);      builder.append(", ");
      builder.append("base=");
      builder.append(this.base_);      builder.append(", ");
      builder.append("ee_motion=");
      builder.append(this.ee_motion_);      builder.append(", ");
      builder.append("ee_forces=");
      builder.append(this.ee_forces_);      builder.append(", ");
      builder.append("ee_contact=");
      builder.append(this.ee_contact_);
      builder.append("}");
      return builder.toString();
   }
}
