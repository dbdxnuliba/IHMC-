package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerParametersMessage extends Packet<StepUpPlannerParametersMessage> implements Settable<StepUpPlannerParametersMessage>, EpsilonComparable<StepUpPlannerParametersMessage>
{
   /**
            * Phases definition
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseParameters>  phases_settings_;
   /**
            * Number of control cycles per phase
            */
   public long phase_length_;
   /**
            * Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
            */
   public byte solver_verbosity_;
   /**
            * Maximum distance between the CoM and the foot
            */
   public double max_leg_length_;
   /**
            * Ipopt internal linear solver
            */
   public java.lang.StringBuilder ipopt_linear_solver_;
   /**
            * Percentage of the last phase in which weighting the error from the desired value
            */
   public double final_state_anticipation_;
   /**
            * Static friction coefficient
            */
   public double static_friction_coefficient_;
   /**
            * Torsional friction coefficient
            */
   public double torsional_friction_coefficient_;
   /**
            * The weights for the cost function
            */
   public controller_msgs.msg.dds.StepUpPlannerCostWeights cost_weights_;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public StepUpPlannerParametersMessage()
   {
      phases_settings_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseParameters> (20, new controller_msgs.msg.dds.StepUpPlannerPhaseParametersPubSubType());
      ipopt_linear_solver_ = new java.lang.StringBuilder(255);
      cost_weights_ = new controller_msgs.msg.dds.StepUpPlannerCostWeights();

   }

   public StepUpPlannerParametersMessage(StepUpPlannerParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerParametersMessage other)
   {
      phases_settings_.set(other.phases_settings_);
      phase_length_ = other.phase_length_;

      solver_verbosity_ = other.solver_verbosity_;

      max_leg_length_ = other.max_leg_length_;

      ipopt_linear_solver_.setLength(0);
      ipopt_linear_solver_.append(other.ipopt_linear_solver_);

      final_state_anticipation_ = other.final_state_anticipation_;

      static_friction_coefficient_ = other.static_friction_coefficient_;

      torsional_friction_coefficient_ = other.torsional_friction_coefficient_;

      controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType.staticCopy(other.cost_weights_, cost_weights_);
      sequence_id_ = other.sequence_id_;

   }


   /**
            * Phases definition
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseParameters>  getPhasesSettings()
   {
      return phases_settings_;
   }

   /**
            * Number of control cycles per phase
            */
   public void setPhaseLength(long phase_length)
   {
      phase_length_ = phase_length;
   }
   /**
            * Number of control cycles per phase
            */
   public long getPhaseLength()
   {
      return phase_length_;
   }

   /**
            * Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
            */
   public void setSolverVerbosity(byte solver_verbosity)
   {
      solver_verbosity_ = solver_verbosity;
   }
   /**
            * Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
            */
   public byte getSolverVerbosity()
   {
      return solver_verbosity_;
   }

   /**
            * Maximum distance between the CoM and the foot
            */
   public void setMaxLegLength(double max_leg_length)
   {
      max_leg_length_ = max_leg_length;
   }
   /**
            * Maximum distance between the CoM and the foot
            */
   public double getMaxLegLength()
   {
      return max_leg_length_;
   }

   /**
            * Ipopt internal linear solver
            */
   public void setIpoptLinearSolver(java.lang.String ipopt_linear_solver)
   {
      ipopt_linear_solver_.setLength(0);
      ipopt_linear_solver_.append(ipopt_linear_solver);
   }

   /**
            * Ipopt internal linear solver
            */
   public java.lang.String getIpoptLinearSolverAsString()
   {
      return getIpoptLinearSolver().toString();
   }
   /**
            * Ipopt internal linear solver
            */
   public java.lang.StringBuilder getIpoptLinearSolver()
   {
      return ipopt_linear_solver_;
   }

   /**
            * Percentage of the last phase in which weighting the error from the desired value
            */
   public void setFinalStateAnticipation(double final_state_anticipation)
   {
      final_state_anticipation_ = final_state_anticipation;
   }
   /**
            * Percentage of the last phase in which weighting the error from the desired value
            */
   public double getFinalStateAnticipation()
   {
      return final_state_anticipation_;
   }

   /**
            * Static friction coefficient
            */
   public void setStaticFrictionCoefficient(double static_friction_coefficient)
   {
      static_friction_coefficient_ = static_friction_coefficient;
   }
   /**
            * Static friction coefficient
            */
   public double getStaticFrictionCoefficient()
   {
      return static_friction_coefficient_;
   }

   /**
            * Torsional friction coefficient
            */
   public void setTorsionalFrictionCoefficient(double torsional_friction_coefficient)
   {
      torsional_friction_coefficient_ = torsional_friction_coefficient;
   }
   /**
            * Torsional friction coefficient
            */
   public double getTorsionalFrictionCoefficient()
   {
      return torsional_friction_coefficient_;
   }


   /**
            * The weights for the cost function
            */
   public controller_msgs.msg.dds.StepUpPlannerCostWeights getCostWeights()
   {
      return cost_weights_;
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


   public static Supplier<StepUpPlannerParametersMessagePubSubType> getPubSubType()
   {
      return StepUpPlannerParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.phases_settings_.size() != other.phases_settings_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.phases_settings_.size(); i++)
         {  if (!this.phases_settings_.get(i).epsilonEquals(other.phases_settings_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.phase_length_, other.phase_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solver_verbosity_, other.solver_verbosity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_leg_length_, other.max_leg_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.ipopt_linear_solver_, other.ipopt_linear_solver_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_state_anticipation_, other.final_state_anticipation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.static_friction_coefficient_, other.static_friction_coefficient_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torsional_friction_coefficient_, other.torsional_friction_coefficient_, epsilon)) return false;

      if (!this.cost_weights_.epsilonEquals(other.cost_weights_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerParametersMessage)) return false;

      StepUpPlannerParametersMessage otherMyClass = (StepUpPlannerParametersMessage) other;

      if (!this.phases_settings_.equals(otherMyClass.phases_settings_)) return false;
      if(this.phase_length_ != otherMyClass.phase_length_) return false;

      if(this.solver_verbosity_ != otherMyClass.solver_verbosity_) return false;

      if(this.max_leg_length_ != otherMyClass.max_leg_length_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.ipopt_linear_solver_, otherMyClass.ipopt_linear_solver_)) return false;

      if(this.final_state_anticipation_ != otherMyClass.final_state_anticipation_) return false;

      if(this.static_friction_coefficient_ != otherMyClass.static_friction_coefficient_) return false;

      if(this.torsional_friction_coefficient_ != otherMyClass.torsional_friction_coefficient_) return false;

      if (!this.cost_weights_.equals(otherMyClass.cost_weights_)) return false;
      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerParametersMessage {");
      builder.append("phases_settings=");
      builder.append(this.phases_settings_);      builder.append(", ");
      builder.append("phase_length=");
      builder.append(this.phase_length_);      builder.append(", ");
      builder.append("solver_verbosity=");
      builder.append(this.solver_verbosity_);      builder.append(", ");
      builder.append("max_leg_length=");
      builder.append(this.max_leg_length_);      builder.append(", ");
      builder.append("ipopt_linear_solver=");
      builder.append(this.ipopt_linear_solver_);      builder.append(", ");
      builder.append("final_state_anticipation=");
      builder.append(this.final_state_anticipation_);      builder.append(", ");
      builder.append("static_friction_coefficient=");
      builder.append(this.static_friction_coefficient_);      builder.append(", ");
      builder.append("torsional_friction_coefficient=");
      builder.append(this.torsional_friction_coefficient_);      builder.append(", ");
      builder.append("cost_weights=");
      builder.append(this.cost_weights_);      builder.append(", ");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
