package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the KinematicsStreamingToolbox API.
       */
public class KinematicsStreamingToolboxCalibrationMessage extends Packet<KinematicsStreamingToolboxCalibrationMessage> implements Settable<KinematicsStreamingToolboxCalibrationMessage>, EpsilonComparable<KinematicsStreamingToolboxCalibrationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public boolean use_ground_height_;
   public boolean use_head_pose_;
   public boolean use_left_hand_pose_;
   public boolean use_right_hand_pose_;
   public double ground_height_;
   public us.ihmc.euclid.geometry.Pose3D head_pose_;
   public us.ihmc.euclid.geometry.Pose3D left_hand_pose_;
   public us.ihmc.euclid.geometry.Pose3D right_hand_pose_;

   public KinematicsStreamingToolboxCalibrationMessage()
   {
      head_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      left_hand_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      right_hand_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public KinematicsStreamingToolboxCalibrationMessage(KinematicsStreamingToolboxCalibrationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxCalibrationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      use_ground_height_ = other.use_ground_height_;

      use_head_pose_ = other.use_head_pose_;

      use_left_hand_pose_ = other.use_left_hand_pose_;

      use_right_hand_pose_ = other.use_right_hand_pose_;

      ground_height_ = other.ground_height_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.head_pose_, head_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.left_hand_pose_, left_hand_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.right_hand_pose_, right_hand_pose_);
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

   public void setUseGroundHeight(boolean use_ground_height)
   {
      use_ground_height_ = use_ground_height;
   }
   public boolean getUseGroundHeight()
   {
      return use_ground_height_;
   }

   public void setUseHeadPose(boolean use_head_pose)
   {
      use_head_pose_ = use_head_pose;
   }
   public boolean getUseHeadPose()
   {
      return use_head_pose_;
   }

   public void setUseLeftHandPose(boolean use_left_hand_pose)
   {
      use_left_hand_pose_ = use_left_hand_pose;
   }
   public boolean getUseLeftHandPose()
   {
      return use_left_hand_pose_;
   }

   public void setUseRightHandPose(boolean use_right_hand_pose)
   {
      use_right_hand_pose_ = use_right_hand_pose;
   }
   public boolean getUseRightHandPose()
   {
      return use_right_hand_pose_;
   }

   public void setGroundHeight(double ground_height)
   {
      ground_height_ = ground_height;
   }
   public double getGroundHeight()
   {
      return ground_height_;
   }


   public us.ihmc.euclid.geometry.Pose3D getHeadPose()
   {
      return head_pose_;
   }


   public us.ihmc.euclid.geometry.Pose3D getLeftHandPose()
   {
      return left_hand_pose_;
   }


   public us.ihmc.euclid.geometry.Pose3D getRightHandPose()
   {
      return right_hand_pose_;
   }


   public static Supplier<KinematicsStreamingToolboxCalibrationMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxCalibrationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxCalibrationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxCalibrationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_ground_height_, other.use_ground_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_head_pose_, other.use_head_pose_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_left_hand_pose_, other.use_left_hand_pose_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_right_hand_pose_, other.use_right_hand_pose_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ground_height_, other.ground_height_, epsilon)) return false;

      if (!this.head_pose_.epsilonEquals(other.head_pose_, epsilon)) return false;
      if (!this.left_hand_pose_.epsilonEquals(other.left_hand_pose_, epsilon)) return false;
      if (!this.right_hand_pose_.epsilonEquals(other.right_hand_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxCalibrationMessage)) return false;

      KinematicsStreamingToolboxCalibrationMessage otherMyClass = (KinematicsStreamingToolboxCalibrationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.use_ground_height_ != otherMyClass.use_ground_height_) return false;

      if(this.use_head_pose_ != otherMyClass.use_head_pose_) return false;

      if(this.use_left_hand_pose_ != otherMyClass.use_left_hand_pose_) return false;

      if(this.use_right_hand_pose_ != otherMyClass.use_right_hand_pose_) return false;

      if(this.ground_height_ != otherMyClass.ground_height_) return false;

      if (!this.head_pose_.equals(otherMyClass.head_pose_)) return false;
      if (!this.left_hand_pose_.equals(otherMyClass.left_hand_pose_)) return false;
      if (!this.right_hand_pose_.equals(otherMyClass.right_hand_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxCalibrationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("use_ground_height=");
      builder.append(this.use_ground_height_);      builder.append(", ");
      builder.append("use_head_pose=");
      builder.append(this.use_head_pose_);      builder.append(", ");
      builder.append("use_left_hand_pose=");
      builder.append(this.use_left_hand_pose_);      builder.append(", ");
      builder.append("use_right_hand_pose=");
      builder.append(this.use_right_hand_pose_);      builder.append(", ");
      builder.append("ground_height=");
      builder.append(this.ground_height_);      builder.append(", ");
      builder.append("head_pose=");
      builder.append(this.head_pose_);      builder.append(", ");
      builder.append("left_hand_pose=");
      builder.append(this.left_hand_pose_);      builder.append(", ");
      builder.append("right_hand_pose=");
      builder.append(this.right_hand_pose_);
      builder.append("}");
      return builder.toString();
   }
}
