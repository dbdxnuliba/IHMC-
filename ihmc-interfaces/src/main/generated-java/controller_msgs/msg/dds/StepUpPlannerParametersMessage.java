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
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseParameters>  phases_parameters_;
   /**
            * Number of control cycles per phase
            */
   public long phase_length_ = 30;
   /**
            * Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
            */
   public long solver_verbosity_ = 1;
   /**
            * Maximum distance between the CoM and the foot
            */
   public double max_leg_length_ = 2.0;
   /**
            * Minimum distance between the CoM and the foot
            */
   public double min_leg_length_;
   /**
            * Ipopt internal linear solver
            */
   public java.lang.StringBuilder ipopt_linear_solver_;
   /**
            * Percentage of the last phase in which weighting the error from the desired value
            */
   public double final_state_anticipation_ = 0.3;
   /**
            * Static friction coefficient
            */
   public double static_friction_coefficient_ = 1.0;
   /**
            * Torsional friction coefficient
            */
   public double torsional_friction_coefficient_ = 1.0;
   /**
            * The weights for the cost function
            */
   public controller_msgs.msg.dds.StepUpPlannerCostWeights cost_weights_;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Include a set of CenterOfMassTrajectoryMessages in the respond
            */
   public boolean include_com_messages_;
   /**
            * Send the CenterOfMassTrajectoryMessages directly (the topic name has also to be specified)
            */
   public boolean send_com_messages_;
   /**
            * Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
            */
   public java.lang.StringBuilder com_messages_topic_;
   /**
            * Maximum number of points to be defined in a single CenterOfMassTrajectoryMessage
            */
   public long max_com_message_length_ = 50;
   /**
            * Include a set of PelvisHeightTrajectoryMessages in the respond
            */
   public boolean include_pelvis_height_messages_;
   /**
            * Send the PelvisHeightTrajectoryMessages directly (the topic name has also to be specified)
            */
   public boolean send_pelvis_height_messages_;
   /**
            * A delta to be added to the CoM height to retrieve the pelvis height
            */
   public double pelvis_height_delta_;
   /**
            * Topic name of IHMC controller accepting the PelvisHeightTrajectoryMessages
            */
   public java.lang.StringBuilder pelvis_height_messages_topic_;
   /**
            * Maximum number of points to be defined in a single PelvisHeightTrajectoryMessages
            */
   public long max_pelvis_height_message_length_ = 50;
   /**
            * Include a set of FootstepDataListMessage as well
            */
   public boolean include_footstep_messages_;
   /**
            * Send a FootstepDataListMessage as well
            */
   public boolean send_footstep_messages_;
   /**
            * Topic name of IHMC controller accepting the FootstepDataListMessage (the topic name has also to be specified)
            */
   public java.lang.StringBuilder footstep_messages_topic_;

   public StepUpPlannerParametersMessage()
   {
      phases_parameters_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseParameters> (20, new controller_msgs.msg.dds.StepUpPlannerPhaseParametersPubSubType());
      ipopt_linear_solver_ = new java.lang.StringBuilder(255);
      cost_weights_ = new controller_msgs.msg.dds.StepUpPlannerCostWeights();
      com_messages_topic_ = new java.lang.StringBuilder(255);
      pelvis_height_messages_topic_ = new java.lang.StringBuilder(255);
      footstep_messages_topic_ = new java.lang.StringBuilder(255);

   }

   public StepUpPlannerParametersMessage(StepUpPlannerParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerParametersMessage other)
   {
      phases_parameters_.set(other.phases_parameters_);
      phase_length_ = other.phase_length_;

      solver_verbosity_ = other.solver_verbosity_;

      max_leg_length_ = other.max_leg_length_;

      min_leg_length_ = other.min_leg_length_;

      ipopt_linear_solver_.setLength(0);
      ipopt_linear_solver_.append(other.ipopt_linear_solver_);

      final_state_anticipation_ = other.final_state_anticipation_;

      static_friction_coefficient_ = other.static_friction_coefficient_;

      torsional_friction_coefficient_ = other.torsional_friction_coefficient_;

      controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType.staticCopy(other.cost_weights_, cost_weights_);
      sequence_id_ = other.sequence_id_;

      include_com_messages_ = other.include_com_messages_;

      send_com_messages_ = other.send_com_messages_;

      com_messages_topic_.setLength(0);
      com_messages_topic_.append(other.com_messages_topic_);

      max_com_message_length_ = other.max_com_message_length_;

      include_pelvis_height_messages_ = other.include_pelvis_height_messages_;

      send_pelvis_height_messages_ = other.send_pelvis_height_messages_;

      pelvis_height_delta_ = other.pelvis_height_delta_;

      pelvis_height_messages_topic_.setLength(0);
      pelvis_height_messages_topic_.append(other.pelvis_height_messages_topic_);

      max_pelvis_height_message_length_ = other.max_pelvis_height_message_length_;

      include_footstep_messages_ = other.include_footstep_messages_;

      send_footstep_messages_ = other.send_footstep_messages_;

      footstep_messages_topic_.setLength(0);
      footstep_messages_topic_.append(other.footstep_messages_topic_);

   }


   /**
            * Phases definition
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerPhaseParameters>  getPhasesParameters()
   {
      return phases_parameters_;
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
   public void setSolverVerbosity(long solver_verbosity)
   {
      solver_verbosity_ = solver_verbosity;
   }
   /**
            * Solver verbosity (0 to disable printing, 1 prints the timing, >1 increase IPOPT printing level
            */
   public long getSolverVerbosity()
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
            * Minimum distance between the CoM and the foot
            */
   public void setMinLegLength(double min_leg_length)
   {
      min_leg_length_ = min_leg_length;
   }
   /**
            * Minimum distance between the CoM and the foot
            */
   public double getMinLegLength()
   {
      return min_leg_length_;
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

   /**
            * Include a set of CenterOfMassTrajectoryMessages in the respond
            */
   public void setIncludeComMessages(boolean include_com_messages)
   {
      include_com_messages_ = include_com_messages;
   }
   /**
            * Include a set of CenterOfMassTrajectoryMessages in the respond
            */
   public boolean getIncludeComMessages()
   {
      return include_com_messages_;
   }

   /**
            * Send the CenterOfMassTrajectoryMessages directly (the topic name has also to be specified)
            */
   public void setSendComMessages(boolean send_com_messages)
   {
      send_com_messages_ = send_com_messages;
   }
   /**
            * Send the CenterOfMassTrajectoryMessages directly (the topic name has also to be specified)
            */
   public boolean getSendComMessages()
   {
      return send_com_messages_;
   }

   /**
            * Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
            */
   public void setComMessagesTopic(java.lang.String com_messages_topic)
   {
      com_messages_topic_.setLength(0);
      com_messages_topic_.append(com_messages_topic);
   }

   /**
            * Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
            */
   public java.lang.String getComMessagesTopicAsString()
   {
      return getComMessagesTopic().toString();
   }
   /**
            * Topic name of IHMC controller accepting the CenterOfMassTrajectoryMessage
            */
   public java.lang.StringBuilder getComMessagesTopic()
   {
      return com_messages_topic_;
   }

   /**
            * Maximum number of points to be defined in a single CenterOfMassTrajectoryMessage
            */
   public void setMaxComMessageLength(long max_com_message_length)
   {
      max_com_message_length_ = max_com_message_length;
   }
   /**
            * Maximum number of points to be defined in a single CenterOfMassTrajectoryMessage
            */
   public long getMaxComMessageLength()
   {
      return max_com_message_length_;
   }

   /**
            * Include a set of PelvisHeightTrajectoryMessages in the respond
            */
   public void setIncludePelvisHeightMessages(boolean include_pelvis_height_messages)
   {
      include_pelvis_height_messages_ = include_pelvis_height_messages;
   }
   /**
            * Include a set of PelvisHeightTrajectoryMessages in the respond
            */
   public boolean getIncludePelvisHeightMessages()
   {
      return include_pelvis_height_messages_;
   }

   /**
            * Send the PelvisHeightTrajectoryMessages directly (the topic name has also to be specified)
            */
   public void setSendPelvisHeightMessages(boolean send_pelvis_height_messages)
   {
      send_pelvis_height_messages_ = send_pelvis_height_messages;
   }
   /**
            * Send the PelvisHeightTrajectoryMessages directly (the topic name has also to be specified)
            */
   public boolean getSendPelvisHeightMessages()
   {
      return send_pelvis_height_messages_;
   }

   /**
            * A delta to be added to the CoM height to retrieve the pelvis height
            */
   public void setPelvisHeightDelta(double pelvis_height_delta)
   {
      pelvis_height_delta_ = pelvis_height_delta;
   }
   /**
            * A delta to be added to the CoM height to retrieve the pelvis height
            */
   public double getPelvisHeightDelta()
   {
      return pelvis_height_delta_;
   }

   /**
            * Topic name of IHMC controller accepting the PelvisHeightTrajectoryMessages
            */
   public void setPelvisHeightMessagesTopic(java.lang.String pelvis_height_messages_topic)
   {
      pelvis_height_messages_topic_.setLength(0);
      pelvis_height_messages_topic_.append(pelvis_height_messages_topic);
   }

   /**
            * Topic name of IHMC controller accepting the PelvisHeightTrajectoryMessages
            */
   public java.lang.String getPelvisHeightMessagesTopicAsString()
   {
      return getPelvisHeightMessagesTopic().toString();
   }
   /**
            * Topic name of IHMC controller accepting the PelvisHeightTrajectoryMessages
            */
   public java.lang.StringBuilder getPelvisHeightMessagesTopic()
   {
      return pelvis_height_messages_topic_;
   }

   /**
            * Maximum number of points to be defined in a single PelvisHeightTrajectoryMessages
            */
   public void setMaxPelvisHeightMessageLength(long max_pelvis_height_message_length)
   {
      max_pelvis_height_message_length_ = max_pelvis_height_message_length;
   }
   /**
            * Maximum number of points to be defined in a single PelvisHeightTrajectoryMessages
            */
   public long getMaxPelvisHeightMessageLength()
   {
      return max_pelvis_height_message_length_;
   }

   /**
            * Include a set of FootstepDataListMessage as well
            */
   public void setIncludeFootstepMessages(boolean include_footstep_messages)
   {
      include_footstep_messages_ = include_footstep_messages;
   }
   /**
            * Include a set of FootstepDataListMessage as well
            */
   public boolean getIncludeFootstepMessages()
   {
      return include_footstep_messages_;
   }

   /**
            * Send a FootstepDataListMessage as well
            */
   public void setSendFootstepMessages(boolean send_footstep_messages)
   {
      send_footstep_messages_ = send_footstep_messages;
   }
   /**
            * Send a FootstepDataListMessage as well
            */
   public boolean getSendFootstepMessages()
   {
      return send_footstep_messages_;
   }

   /**
            * Topic name of IHMC controller accepting the FootstepDataListMessage (the topic name has also to be specified)
            */
   public void setFootstepMessagesTopic(java.lang.String footstep_messages_topic)
   {
      footstep_messages_topic_.setLength(0);
      footstep_messages_topic_.append(footstep_messages_topic);
   }

   /**
            * Topic name of IHMC controller accepting the FootstepDataListMessage (the topic name has also to be specified)
            */
   public java.lang.String getFootstepMessagesTopicAsString()
   {
      return getFootstepMessagesTopic().toString();
   }
   /**
            * Topic name of IHMC controller accepting the FootstepDataListMessage (the topic name has also to be specified)
            */
   public java.lang.StringBuilder getFootstepMessagesTopic()
   {
      return footstep_messages_topic_;
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

      if (this.phases_parameters_.size() != other.phases_parameters_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.phases_parameters_.size(); i++)
         {  if (!this.phases_parameters_.get(i).epsilonEquals(other.phases_parameters_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.phase_length_, other.phase_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solver_verbosity_, other.solver_verbosity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_leg_length_, other.max_leg_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_leg_length_, other.min_leg_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.ipopt_linear_solver_, other.ipopt_linear_solver_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.final_state_anticipation_, other.final_state_anticipation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.static_friction_coefficient_, other.static_friction_coefficient_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torsional_friction_coefficient_, other.torsional_friction_coefficient_, epsilon)) return false;

      if (!this.cost_weights_.epsilonEquals(other.cost_weights_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.include_com_messages_, other.include_com_messages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.send_com_messages_, other.send_com_messages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.com_messages_topic_, other.com_messages_topic_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_com_message_length_, other.max_com_message_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.include_pelvis_height_messages_, other.include_pelvis_height_messages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.send_pelvis_height_messages_, other.send_pelvis_height_messages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pelvis_height_delta_, other.pelvis_height_delta_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.pelvis_height_messages_topic_, other.pelvis_height_messages_topic_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_pelvis_height_message_length_, other.max_pelvis_height_message_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.include_footstep_messages_, other.include_footstep_messages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.send_footstep_messages_, other.send_footstep_messages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.footstep_messages_topic_, other.footstep_messages_topic_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerParametersMessage)) return false;

      StepUpPlannerParametersMessage otherMyClass = (StepUpPlannerParametersMessage) other;

      if (!this.phases_parameters_.equals(otherMyClass.phases_parameters_)) return false;
      if(this.phase_length_ != otherMyClass.phase_length_) return false;

      if(this.solver_verbosity_ != otherMyClass.solver_verbosity_) return false;

      if(this.max_leg_length_ != otherMyClass.max_leg_length_) return false;

      if(this.min_leg_length_ != otherMyClass.min_leg_length_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.ipopt_linear_solver_, otherMyClass.ipopt_linear_solver_)) return false;

      if(this.final_state_anticipation_ != otherMyClass.final_state_anticipation_) return false;

      if(this.static_friction_coefficient_ != otherMyClass.static_friction_coefficient_) return false;

      if(this.torsional_friction_coefficient_ != otherMyClass.torsional_friction_coefficient_) return false;

      if (!this.cost_weights_.equals(otherMyClass.cost_weights_)) return false;
      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.include_com_messages_ != otherMyClass.include_com_messages_) return false;

      if(this.send_com_messages_ != otherMyClass.send_com_messages_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.com_messages_topic_, otherMyClass.com_messages_topic_)) return false;

      if(this.max_com_message_length_ != otherMyClass.max_com_message_length_) return false;

      if(this.include_pelvis_height_messages_ != otherMyClass.include_pelvis_height_messages_) return false;

      if(this.send_pelvis_height_messages_ != otherMyClass.send_pelvis_height_messages_) return false;

      if(this.pelvis_height_delta_ != otherMyClass.pelvis_height_delta_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.pelvis_height_messages_topic_, otherMyClass.pelvis_height_messages_topic_)) return false;

      if(this.max_pelvis_height_message_length_ != otherMyClass.max_pelvis_height_message_length_) return false;

      if(this.include_footstep_messages_ != otherMyClass.include_footstep_messages_) return false;

      if(this.send_footstep_messages_ != otherMyClass.send_footstep_messages_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.footstep_messages_topic_, otherMyClass.footstep_messages_topic_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerParametersMessage {");
      builder.append("phases_parameters=");
      builder.append(this.phases_parameters_);      builder.append(", ");
      builder.append("phase_length=");
      builder.append(this.phase_length_);      builder.append(", ");
      builder.append("solver_verbosity=");
      builder.append(this.solver_verbosity_);      builder.append(", ");
      builder.append("max_leg_length=");
      builder.append(this.max_leg_length_);      builder.append(", ");
      builder.append("min_leg_length=");
      builder.append(this.min_leg_length_);      builder.append(", ");
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
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("include_com_messages=");
      builder.append(this.include_com_messages_);      builder.append(", ");
      builder.append("send_com_messages=");
      builder.append(this.send_com_messages_);      builder.append(", ");
      builder.append("com_messages_topic=");
      builder.append(this.com_messages_topic_);      builder.append(", ");
      builder.append("max_com_message_length=");
      builder.append(this.max_com_message_length_);      builder.append(", ");
      builder.append("include_pelvis_height_messages=");
      builder.append(this.include_pelvis_height_messages_);      builder.append(", ");
      builder.append("send_pelvis_height_messages=");
      builder.append(this.send_pelvis_height_messages_);      builder.append(", ");
      builder.append("pelvis_height_delta=");
      builder.append(this.pelvis_height_delta_);      builder.append(", ");
      builder.append("pelvis_height_messages_topic=");
      builder.append(this.pelvis_height_messages_topic_);      builder.append(", ");
      builder.append("max_pelvis_height_message_length=");
      builder.append(this.max_pelvis_height_message_length_);      builder.append(", ");
      builder.append("include_footstep_messages=");
      builder.append(this.include_footstep_messages_);      builder.append(", ");
      builder.append("send_footstep_messages=");
      builder.append(this.send_footstep_messages_);      builder.append(", ");
      builder.append("footstep_messages_topic=");
      builder.append(this.footstep_messages_topic_);
      builder.append("}");
      return builder.toString();
   }
}
