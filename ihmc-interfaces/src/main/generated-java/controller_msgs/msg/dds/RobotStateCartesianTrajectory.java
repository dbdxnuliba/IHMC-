package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The state of a robot expressed in the cartesian frame
       */
public class RobotStateCartesianTrajectory extends Packet<RobotStateCartesianTrajectory> implements Settable<RobotStateCartesianTrajectory>, EpsilonComparable<RobotStateCartesianTrajectory>
{
   /**
            * The header is used to specify the coordinate frame and the reference time for the trajectory durations
            */
   public std_msgs.msg.dds.Header header_;
   /**
            * A representation of a Cartesian trajectory
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.RobotStateCartesian>  points_;

   public RobotStateCartesianTrajectory()
   {
      header_ = new std_msgs.msg.dds.Header();
      points_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.RobotStateCartesian> (100, new controller_msgs.msg.dds.RobotStateCartesianPubSubType());

   }

   public RobotStateCartesianTrajectory(RobotStateCartesianTrajectory other)
   {
      this();
      set(other);
   }

   public void set(RobotStateCartesianTrajectory other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      points_.set(other.points_);
   }


   /**
            * The header is used to specify the coordinate frame and the reference time for the trajectory durations
            */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * A representation of a Cartesian trajectory
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.RobotStateCartesian>  getPoints()
   {
      return points_;
   }


   public static Supplier<RobotStateCartesianTrajectoryPubSubType> getPubSubType()
   {
      return RobotStateCartesianTrajectoryPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RobotStateCartesianTrajectoryPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RobotStateCartesianTrajectory other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.points_.size() != other.points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.points_.size(); i++)
         {  if (!this.points_.get(i).epsilonEquals(other.points_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RobotStateCartesianTrajectory)) return false;

      RobotStateCartesianTrajectory otherMyClass = (RobotStateCartesianTrajectory) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.points_.equals(otherMyClass.points_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotStateCartesianTrajectory {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("points=");
      builder.append(this.points_);
      builder.append("}");
      return builder.toString();
   }
}
