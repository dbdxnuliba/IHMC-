package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerCostWeights extends Packet<StepUpPlannerCostWeights> implements Settable<StepUpPlannerCostWeights>, EpsilonComparable<StepUpPlannerCostWeights>
{
   /**
            * CoP weight
            */
   public double cop_ = 1.0;
   /**
            * Weight for a proxy of the exerted torques
            */
   public double torques_ = 1.0;
   /**
            * Weight for a proxy of the maximum exerted torques
            */
   public double max_torques_ = 1.0;
   /**
            * Weight on the control amplitude
            */
   public double control_multipliers_ = 1.0;
   /**
            * Weight on the final control value
            */
   public double final_control_ = 1.0;
   /**
            * Weight on the maximum control amplitude
            */
   public double max_control_multiplier_ = 1.0;
   /**
            * Weight on the final state error
            */
   public double final_state_ = 10.0;
   /**
            * Weight on the control variation
            */
   public double control_variations_ = 1.0;
   /**
            * Weight on the difference from the desired phase duration
            */
   public double durations_difference_ = 1.0;

   public StepUpPlannerCostWeights()
   {
   }

   public StepUpPlannerCostWeights(StepUpPlannerCostWeights other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerCostWeights other)
   {
      cop_ = other.cop_;

      torques_ = other.torques_;

      max_torques_ = other.max_torques_;

      control_multipliers_ = other.control_multipliers_;

      final_control_ = other.final_control_;

      max_control_multiplier_ = other.max_control_multiplier_;

      final_state_ = other.final_state_;

      control_variations_ = other.control_variations_;

      durations_difference_ = other.durations_difference_;

   }

   /**
            * CoP weight
            */
   public void setCop(double cop)
   {
      cop_ = cop;
   }
   /**
            * CoP weight
            */
   public double getCop()
   {
      return cop_;
   }

   /**
            * Weight for a proxy of the exerted torques
            */
   public void setTorques(double torques)
   {
      torques_ = torques;
   }
   /**
            * Weight for a proxy of the exerted torques
            */
   public double getTorques()
   {
      return torques_;
   }

   /**
            * Weight for a proxy of the maximum exerted torques
            */
   public void setMaxTorques(double max_torques)
   {
      max_torques_ = max_torques;
   }
   /**
            * Weight for a proxy of the maximum exerted torques
            */
   public double getMaxTorques()
   {
      return max_torques_;
   }

   /**
            * Weight on the control amplitude
            */
   public void setControlMultipliers(double control_multipliers)
   {
      control_multipliers_ = control_multipliers;
   }
   /**
            * Weight on the control amplitude
            */
   public double getControlMultipliers()
   {
      return control_multipliers_;
   }

   /**
            * Weight on the final control value
            */
   public void setFinalControl(double final_control)
   {
      final_control_ = final_control;
   }
   /**
            * Weight on the final control value
            */
   public double getFinalControl()
   {
      return final_control_;
   }

   /**
            * Weight on the maximum control amplitude
            */
   public void setMaxControlMultiplier(double max_control_multiplier)
   {
      max_control_multiplier_ = max_control_multiplier;
   }
   /**
            * Weight on the maximum control amplitude
            */
   public double getMaxControlMultiplier()
   {
      return max_control_multiplier_;
   }

   /**
            * Weight on the final state error
            */
   public void setFinalState(double final_state)
   {
      final_state_ = final_state;
   }
   /**
            * Weight on the final state error
            */
   public double getFinalState()
   {
      return final_state_;
   }

   /**
            * Weight on the control variation
            */
   public void setControlVariations(double control_variations)
   {
      control_variations_ = control_variations;
   }
   /**
            * Weight on the control variation
            */
   public double getControlVariations()
   {
      return control_variations_;
   }

   /**
            * Weight on the difference from the desired phase duration
            */
   public void setDurationsDifference(double durations_difference)
   {
      durations_difference_ = durations_difference;
   }
   /**
            * Weight on the difference from the desired phase duration
            */
   public double getDurationsDifference()
   {
      return durations_difference_;
   }


   public static Supplier<StepUpPlannerCostWeightsPubSubType> getPubSubType()
   {
      return StepUpPlannerCostWeightsPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerCostWeightsPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerCostWeights other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cop_, other.cop_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torques_, other.torques_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_torques_, other.max_torques_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.control_multipliers_, other.control_multipliers_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_control_, other.final_control_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_control_multiplier_, other.max_control_multiplier_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_state_, other.final_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.control_variations_, other.control_variations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.durations_difference_, other.durations_difference_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerCostWeights)) return false;

      StepUpPlannerCostWeights otherMyClass = (StepUpPlannerCostWeights) other;

      if(this.cop_ != otherMyClass.cop_) return false;

      if(this.torques_ != otherMyClass.torques_) return false;

      if(this.max_torques_ != otherMyClass.max_torques_) return false;

      if(this.control_multipliers_ != otherMyClass.control_multipliers_) return false;

      if(this.final_control_ != otherMyClass.final_control_) return false;

      if(this.max_control_multiplier_ != otherMyClass.max_control_multiplier_) return false;

      if(this.final_state_ != otherMyClass.final_state_) return false;

      if(this.control_variations_ != otherMyClass.control_variations_) return false;

      if(this.durations_difference_ != otherMyClass.durations_difference_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerCostWeights {");
      builder.append("cop=");
      builder.append(this.cop_);      builder.append(", ");
      builder.append("torques=");
      builder.append(this.torques_);      builder.append(", ");
      builder.append("max_torques=");
      builder.append(this.max_torques_);      builder.append(", ");
      builder.append("control_multipliers=");
      builder.append(this.control_multipliers_);      builder.append(", ");
      builder.append("final_control=");
      builder.append(this.final_control_);      builder.append(", ");
      builder.append("max_control_multiplier=");
      builder.append(this.max_control_multiplier_);      builder.append(", ");
      builder.append("final_state=");
      builder.append(this.final_state_);      builder.append(", ");
      builder.append("control_variations=");
      builder.append(this.control_variations_);      builder.append(", ");
      builder.append("durations_difference=");
      builder.append(this.durations_difference_);
      builder.append("}");
      return builder.toString();
   }
}
