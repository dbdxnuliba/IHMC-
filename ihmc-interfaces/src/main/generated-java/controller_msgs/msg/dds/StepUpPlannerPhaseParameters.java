package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerPhaseParameters extends Packet<StepUpPlannerPhaseParameters> implements Settable<StepUpPlannerPhaseParameters>, EpsilonComparable<StepUpPlannerPhaseParameters>
{
   /**
            * Step settings
            */
   public controller_msgs.msg.dds.StepUpPlannerStepParameters left_step_parameters_;
   public controller_msgs.msg.dds.StepUpPlannerStepParameters right_step_parameters_;

   public StepUpPlannerPhaseParameters()
   {
      left_step_parameters_ = new controller_msgs.msg.dds.StepUpPlannerStepParameters();
      right_step_parameters_ = new controller_msgs.msg.dds.StepUpPlannerStepParameters();
   }

   public StepUpPlannerPhaseParameters(StepUpPlannerPhaseParameters other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerPhaseParameters other)
   {
      controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.staticCopy(other.left_step_parameters_, left_step_parameters_);
      controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.staticCopy(other.right_step_parameters_, right_step_parameters_);
   }


   /**
            * Step settings
            */
   public controller_msgs.msg.dds.StepUpPlannerStepParameters getLeftStepParameters()
   {
      return left_step_parameters_;
   }


   public controller_msgs.msg.dds.StepUpPlannerStepParameters getRightStepParameters()
   {
      return right_step_parameters_;
   }


   public static Supplier<StepUpPlannerPhaseParametersPubSubType> getPubSubType()
   {
      return StepUpPlannerPhaseParametersPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerPhaseParametersPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerPhaseParameters other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.left_step_parameters_.epsilonEquals(other.left_step_parameters_, epsilon)) return false;
      if (!this.right_step_parameters_.epsilonEquals(other.right_step_parameters_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerPhaseParameters)) return false;

      StepUpPlannerPhaseParameters otherMyClass = (StepUpPlannerPhaseParameters) other;

      if (!this.left_step_parameters_.equals(otherMyClass.left_step_parameters_)) return false;
      if (!this.right_step_parameters_.equals(otherMyClass.right_step_parameters_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerPhaseParameters {");
      builder.append("left_step_parameters=");
      builder.append(this.left_step_parameters_);      builder.append(", ");
      builder.append("right_step_parameters=");
      builder.append(this.right_step_parameters_);
      builder.append("}");
      return builder.toString();
   }
}
