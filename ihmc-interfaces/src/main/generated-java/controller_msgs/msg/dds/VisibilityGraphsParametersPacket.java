package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class VisibilityGraphsParametersPacket extends Packet<VisibilityGraphsParametersPacket> implements Settable<VisibilityGraphsParametersPacket>, EpsilonComparable<VisibilityGraphsParametersPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double max_inter_region_connection_length_ = -1.0;
   public double normal_z_threshold_for_accessible_regions_ = -1.0;
   public double extrusion_distance_ = -1.0;
   public double extrusion_distance_if_not_too_high_to_step_ = -1.0;
   public double too_high_to_step_distance_ = -1.0;
   public double cluster_resolution_ = -1.0;
   public double exploration_distance_from_start_goal_ = -1.0;
   public double planar_region_min_area_;
   public long planar_region_min_size_;
   /**
            * Defines the angle from which two regions are considered orthogonal.
            * It is used to determine if a region should be projected onto another as a polygon or a line.
            * It should be close to 90 degrees.
            * Returns the angle threshold to use to determine if a line or polygon projection method should be used.
            */
   public double region_orthogonal_angle_ = -1.0;
   /**
            * This epsilon is is used when searching to which region the start/goal belongs to.
            * A positive value corresponds to growing all the regions before testing if the start/goal is inside.
            * Returns the value of the epsilon to use.
            */
   public double search_host_region_epsilon_ = 0.03;

   public VisibilityGraphsParametersPacket()
   {
   }

   public VisibilityGraphsParametersPacket(VisibilityGraphsParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(VisibilityGraphsParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      max_inter_region_connection_length_ = other.max_inter_region_connection_length_;

      normal_z_threshold_for_accessible_regions_ = other.normal_z_threshold_for_accessible_regions_;

      extrusion_distance_ = other.extrusion_distance_;

      extrusion_distance_if_not_too_high_to_step_ = other.extrusion_distance_if_not_too_high_to_step_;

      too_high_to_step_distance_ = other.too_high_to_step_distance_;

      cluster_resolution_ = other.cluster_resolution_;

      exploration_distance_from_start_goal_ = other.exploration_distance_from_start_goal_;

      planar_region_min_area_ = other.planar_region_min_area_;

      planar_region_min_size_ = other.planar_region_min_size_;

      region_orthogonal_angle_ = other.region_orthogonal_angle_;

      search_host_region_epsilon_ = other.search_host_region_epsilon_;

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

   public void setMaxInterRegionConnectionLength(double max_inter_region_connection_length)
   {
      max_inter_region_connection_length_ = max_inter_region_connection_length;
   }
   public double getMaxInterRegionConnectionLength()
   {
      return max_inter_region_connection_length_;
   }

   public void setNormalZThresholdForAccessibleRegions(double normal_z_threshold_for_accessible_regions)
   {
      normal_z_threshold_for_accessible_regions_ = normal_z_threshold_for_accessible_regions;
   }
   public double getNormalZThresholdForAccessibleRegions()
   {
      return normal_z_threshold_for_accessible_regions_;
   }

   public void setExtrusionDistance(double extrusion_distance)
   {
      extrusion_distance_ = extrusion_distance;
   }
   public double getExtrusionDistance()
   {
      return extrusion_distance_;
   }

   public void setExtrusionDistanceIfNotTooHighToStep(double extrusion_distance_if_not_too_high_to_step)
   {
      extrusion_distance_if_not_too_high_to_step_ = extrusion_distance_if_not_too_high_to_step;
   }
   public double getExtrusionDistanceIfNotTooHighToStep()
   {
      return extrusion_distance_if_not_too_high_to_step_;
   }

   public void setTooHighToStepDistance(double too_high_to_step_distance)
   {
      too_high_to_step_distance_ = too_high_to_step_distance;
   }
   public double getTooHighToStepDistance()
   {
      return too_high_to_step_distance_;
   }

   public void setClusterResolution(double cluster_resolution)
   {
      cluster_resolution_ = cluster_resolution;
   }
   public double getClusterResolution()
   {
      return cluster_resolution_;
   }

   public void setExplorationDistanceFromStartGoal(double exploration_distance_from_start_goal)
   {
      exploration_distance_from_start_goal_ = exploration_distance_from_start_goal;
   }
   public double getExplorationDistanceFromStartGoal()
   {
      return exploration_distance_from_start_goal_;
   }

   public void setPlanarRegionMinArea(double planar_region_min_area)
   {
      planar_region_min_area_ = planar_region_min_area;
   }
   public double getPlanarRegionMinArea()
   {
      return planar_region_min_area_;
   }

   public void setPlanarRegionMinSize(long planar_region_min_size)
   {
      planar_region_min_size_ = planar_region_min_size;
   }
   public long getPlanarRegionMinSize()
   {
      return planar_region_min_size_;
   }

   /**
            * Defines the angle from which two regions are considered orthogonal.
            * It is used to determine if a region should be projected onto another as a polygon or a line.
            * It should be close to 90 degrees.
            * Returns the angle threshold to use to determine if a line or polygon projection method should be used.
            */
   public void setRegionOrthogonalAngle(double region_orthogonal_angle)
   {
      region_orthogonal_angle_ = region_orthogonal_angle;
   }
   /**
            * Defines the angle from which two regions are considered orthogonal.
            * It is used to determine if a region should be projected onto another as a polygon or a line.
            * It should be close to 90 degrees.
            * Returns the angle threshold to use to determine if a line or polygon projection method should be used.
            */
   public double getRegionOrthogonalAngle()
   {
      return region_orthogonal_angle_;
   }

   /**
            * This epsilon is is used when searching to which region the start/goal belongs to.
            * A positive value corresponds to growing all the regions before testing if the start/goal is inside.
            * Returns the value of the epsilon to use.
            */
   public void setSearchHostRegionEpsilon(double search_host_region_epsilon)
   {
      search_host_region_epsilon_ = search_host_region_epsilon;
   }
   /**
            * This epsilon is is used when searching to which region the start/goal belongs to.
            * A positive value corresponds to growing all the regions before testing if the start/goal is inside.
            * Returns the value of the epsilon to use.
            */
   public double getSearchHostRegionEpsilon()
   {
      return search_host_region_epsilon_;
   }


   public static Supplier<VisibilityGraphsParametersPacketPubSubType> getPubSubType()
   {
      return VisibilityGraphsParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisibilityGraphsParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisibilityGraphsParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_inter_region_connection_length_, other.max_inter_region_connection_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normal_z_threshold_for_accessible_regions_, other.normal_z_threshold_for_accessible_regions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extrusion_distance_, other.extrusion_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extrusion_distance_if_not_too_high_to_step_, other.extrusion_distance_if_not_too_high_to_step_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.too_high_to_step_distance_, other.too_high_to_step_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cluster_resolution_, other.cluster_resolution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.exploration_distance_from_start_goal_, other.exploration_distance_from_start_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planar_region_min_area_, other.planar_region_min_area_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planar_region_min_size_, other.planar_region_min_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.region_orthogonal_angle_, other.region_orthogonal_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.search_host_region_epsilon_, other.search_host_region_epsilon_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisibilityGraphsParametersPacket)) return false;

      VisibilityGraphsParametersPacket otherMyClass = (VisibilityGraphsParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.max_inter_region_connection_length_ != otherMyClass.max_inter_region_connection_length_) return false;

      if(this.normal_z_threshold_for_accessible_regions_ != otherMyClass.normal_z_threshold_for_accessible_regions_) return false;

      if(this.extrusion_distance_ != otherMyClass.extrusion_distance_) return false;

      if(this.extrusion_distance_if_not_too_high_to_step_ != otherMyClass.extrusion_distance_if_not_too_high_to_step_) return false;

      if(this.too_high_to_step_distance_ != otherMyClass.too_high_to_step_distance_) return false;

      if(this.cluster_resolution_ != otherMyClass.cluster_resolution_) return false;

      if(this.exploration_distance_from_start_goal_ != otherMyClass.exploration_distance_from_start_goal_) return false;

      if(this.planar_region_min_area_ != otherMyClass.planar_region_min_area_) return false;

      if(this.planar_region_min_size_ != otherMyClass.planar_region_min_size_) return false;

      if(this.region_orthogonal_angle_ != otherMyClass.region_orthogonal_angle_) return false;

      if(this.search_host_region_epsilon_ != otherMyClass.search_host_region_epsilon_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisibilityGraphsParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("max_inter_region_connection_length=");
      builder.append(this.max_inter_region_connection_length_);      builder.append(", ");
      builder.append("normal_z_threshold_for_accessible_regions=");
      builder.append(this.normal_z_threshold_for_accessible_regions_);      builder.append(", ");
      builder.append("extrusion_distance=");
      builder.append(this.extrusion_distance_);      builder.append(", ");
      builder.append("extrusion_distance_if_not_too_high_to_step=");
      builder.append(this.extrusion_distance_if_not_too_high_to_step_);      builder.append(", ");
      builder.append("too_high_to_step_distance=");
      builder.append(this.too_high_to_step_distance_);      builder.append(", ");
      builder.append("cluster_resolution=");
      builder.append(this.cluster_resolution_);      builder.append(", ");
      builder.append("exploration_distance_from_start_goal=");
      builder.append(this.exploration_distance_from_start_goal_);      builder.append(", ");
      builder.append("planar_region_min_area=");
      builder.append(this.planar_region_min_area_);      builder.append(", ");
      builder.append("planar_region_min_size=");
      builder.append(this.planar_region_min_size_);      builder.append(", ");
      builder.append("region_orthogonal_angle=");
      builder.append(this.region_orthogonal_angle_);      builder.append(", ");
      builder.append("search_host_region_epsilon=");
      builder.append(this.search_host_region_epsilon_);
      builder.append("}");
      return builder.toString();
   }
}
