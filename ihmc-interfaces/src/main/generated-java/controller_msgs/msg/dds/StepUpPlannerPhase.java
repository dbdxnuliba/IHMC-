package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerPhase extends Packet<StepUpPlannerPhase> implements Settable<StepUpPlannerPhase>, EpsilonComparable<StepUpPlannerPhase>
{
   /**
            * Position and orientation of the left foot. In case of SWING state this is ignored
            */
   public us.ihmc.euclid.geometry.Pose3D left_foot_pose_;
   /**
            * Position and orientation of the left foot. In case of SWING state this is ignored
            */
   public us.ihmc.euclid.geometry.Pose3D right_foot_pose_;
   /**
            * Phase desired duration
            */
   public double desired_duration_;
   /**
            * Phase minimum duration
            */
   public double minimum_duration_;
   /**
            * Phase maximum duration
            */
   public double maximum_duration_;

   public StepUpPlannerPhase()
   {
      left_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      right_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public StepUpPlannerPhase(StepUpPlannerPhase other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerPhase other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.left_foot_pose_, left_foot_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.right_foot_pose_, right_foot_pose_);
      desired_duration_ = other.desired_duration_;

      minimum_duration_ = other.minimum_duration_;

      maximum_duration_ = other.maximum_duration_;

   }


   /**
            * Position and orientation of the left foot. In case of SWING state this is ignored
            */
   public us.ihmc.euclid.geometry.Pose3D getLeftFootPose()
   {
      return left_foot_pose_;
   }


   /**
            * Position and orientation of the left foot. In case of SWING state this is ignored
            */
   public us.ihmc.euclid.geometry.Pose3D getRightFootPose()
   {
      return right_foot_pose_;
   }

   /**
            * Phase desired duration
            */
   public void setDesiredDuration(double desired_duration)
   {
      desired_duration_ = desired_duration;
   }
   /**
            * Phase desired duration
            */
   public double getDesiredDuration()
   {
      return desired_duration_;
   }

   /**
            * Phase minimum duration
            */
   public void setMinimumDuration(double minimum_duration)
   {
      minimum_duration_ = minimum_duration;
   }
   /**
            * Phase minimum duration
            */
   public double getMinimumDuration()
   {
      return minimum_duration_;
   }

   /**
            * Phase maximum duration
            */
   public void setMaximumDuration(double maximum_duration)
   {
      maximum_duration_ = maximum_duration;
   }
   /**
            * Phase maximum duration
            */
   public double getMaximumDuration()
   {
      return maximum_duration_;
   }


   public static Supplier<StepUpPlannerPhasePubSubType> getPubSubType()
   {
      return StepUpPlannerPhasePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerPhasePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerPhase other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.left_foot_pose_.epsilonEquals(other.left_foot_pose_, epsilon)) return false;
      if (!this.right_foot_pose_.epsilonEquals(other.right_foot_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_duration_, other.desired_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_duration_, other.minimum_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_duration_, other.maximum_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerPhase)) return false;

      StepUpPlannerPhase otherMyClass = (StepUpPlannerPhase) other;

      if (!this.left_foot_pose_.equals(otherMyClass.left_foot_pose_)) return false;
      if (!this.right_foot_pose_.equals(otherMyClass.right_foot_pose_)) return false;
      if(this.desired_duration_ != otherMyClass.desired_duration_) return false;

      if(this.minimum_duration_ != otherMyClass.minimum_duration_) return false;

      if(this.maximum_duration_ != otherMyClass.maximum_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerPhase {");
      builder.append("left_foot_pose=");
      builder.append(this.left_foot_pose_);      builder.append(", ");
      builder.append("right_foot_pose=");
      builder.append(this.right_foot_pose_);      builder.append(", ");
      builder.append("desired_duration=");
      builder.append(this.desired_duration_);      builder.append(", ");
      builder.append("minimum_duration=");
      builder.append(this.minimum_duration_);      builder.append(", ");
      builder.append("maximum_duration=");
      builder.append(this.maximum_duration_);
      builder.append("}");
      return builder.toString();
   }
}
