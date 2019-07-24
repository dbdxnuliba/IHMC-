package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerParametersMessage" defined in "StepUpPlannerParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerParametersMessage_.idl instead.
*
*/
public class StepUpPlannerParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerParametersMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerParametersMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerPhaseParametersPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPhasesSettings().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerPhaseParametersPubSubType.getCdrSerializedSize(data.getPhasesSettings().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIpoptLinearSolver().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType.getCdrSerializedSize(data.getCostWeights(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getComMessagesTopic().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFootstepMessagesTopic().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPhasesSettings().size() <= 20)
      cdr.write_type_e(data.getPhasesSettings());else
          throw new RuntimeException("phases_settings field exceeds the maximum length");

      cdr.write_type_12(data.getPhaseLength());

      cdr.write_type_12(data.getSolverVerbosity());

      cdr.write_type_6(data.getMaxLegLength());

      if(data.getIpoptLinearSolver().length() <= 255)
      cdr.write_type_d(data.getIpoptLinearSolver());else
          throw new RuntimeException("ipopt_linear_solver field exceeds the maximum length");

      cdr.write_type_6(data.getFinalStateAnticipation());

      cdr.write_type_6(data.getStaticFrictionCoefficient());

      cdr.write_type_6(data.getTorsionalFrictionCoefficient());

      controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType.write(data.getCostWeights(), cdr);
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIncludeComMessages());

      cdr.write_type_7(data.getSendComMessages());

      if(data.getComMessagesTopic().length() <= 255)
      cdr.write_type_d(data.getComMessagesTopic());else
          throw new RuntimeException("com_messages_topic field exceeds the maximum length");

      cdr.write_type_12(data.getMaxComMessageLength());

      cdr.write_type_7(data.getIncludeFootstepMessages());

      cdr.write_type_7(data.getSendFootstepMessages());

      if(data.getFootstepMessagesTopic().length() <= 255)
      cdr.write_type_d(data.getFootstepMessagesTopic());else
          throw new RuntimeException("footstep_messages_topic field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPhasesSettings());	
      data.setPhaseLength(cdr.read_type_12());
      	
      data.setSolverVerbosity(cdr.read_type_12());
      	
      data.setMaxLegLength(cdr.read_type_6());
      	
      cdr.read_type_d(data.getIpoptLinearSolver());	
      data.setFinalStateAnticipation(cdr.read_type_6());
      	
      data.setStaticFrictionCoefficient(cdr.read_type_6());
      	
      data.setTorsionalFrictionCoefficient(cdr.read_type_6());
      	
      controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType.read(data.getCostWeights(), cdr);	
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIncludeComMessages(cdr.read_type_7());
      	
      data.setSendComMessages(cdr.read_type_7());
      	
      cdr.read_type_d(data.getComMessagesTopic());	
      data.setMaxComMessageLength(cdr.read_type_12());
      	
      data.setIncludeFootstepMessages(cdr.read_type_7());
      	
      data.setSendFootstepMessages(cdr.read_type_7());
      	
      cdr.read_type_d(data.getFootstepMessagesTopic());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("phases_settings", data.getPhasesSettings());
      ser.write_type_12("phase_length", data.getPhaseLength());
      ser.write_type_12("solver_verbosity", data.getSolverVerbosity());
      ser.write_type_6("max_leg_length", data.getMaxLegLength());
      ser.write_type_d("ipopt_linear_solver", data.getIpoptLinearSolver());
      ser.write_type_6("final_state_anticipation", data.getFinalStateAnticipation());
      ser.write_type_6("static_friction_coefficient", data.getStaticFrictionCoefficient());
      ser.write_type_6("torsional_friction_coefficient", data.getTorsionalFrictionCoefficient());
      ser.write_type_a("cost_weights", new controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType(), data.getCostWeights());

      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("include_com_messages", data.getIncludeComMessages());
      ser.write_type_7("send_com_messages", data.getSendComMessages());
      ser.write_type_d("com_messages_topic", data.getComMessagesTopic());
      ser.write_type_12("max_com_message_length", data.getMaxComMessageLength());
      ser.write_type_7("include_footstep_messages", data.getIncludeFootstepMessages());
      ser.write_type_7("send_footstep_messages", data.getSendFootstepMessages());
      ser.write_type_d("footstep_messages_topic", data.getFootstepMessagesTopic());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerParametersMessage data)
   {
      ser.read_type_e("phases_settings", data.getPhasesSettings());
      data.setPhaseLength(ser.read_type_12("phase_length"));
      data.setSolverVerbosity(ser.read_type_12("solver_verbosity"));
      data.setMaxLegLength(ser.read_type_6("max_leg_length"));
      ser.read_type_d("ipopt_linear_solver", data.getIpoptLinearSolver());
      data.setFinalStateAnticipation(ser.read_type_6("final_state_anticipation"));
      data.setStaticFrictionCoefficient(ser.read_type_6("static_friction_coefficient"));
      data.setTorsionalFrictionCoefficient(ser.read_type_6("torsional_friction_coefficient"));
      ser.read_type_a("cost_weights", new controller_msgs.msg.dds.StepUpPlannerCostWeightsPubSubType(), data.getCostWeights());

      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIncludeComMessages(ser.read_type_7("include_com_messages"));
      data.setSendComMessages(ser.read_type_7("send_com_messages"));
      ser.read_type_d("com_messages_topic", data.getComMessagesTopic());
      data.setMaxComMessageLength(ser.read_type_12("max_com_message_length"));
      data.setIncludeFootstepMessages(ser.read_type_7("include_footstep_messages"));
      data.setSendFootstepMessages(ser.read_type_7("send_footstep_messages"));
      ser.read_type_d("footstep_messages_topic", data.getFootstepMessagesTopic());
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerParametersMessage src, controller_msgs.msg.dds.StepUpPlannerParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerParametersMessage createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerParametersMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerParametersMessage src, controller_msgs.msg.dds.StepUpPlannerParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerParametersMessagePubSubType newInstance()
   {
      return new StepUpPlannerParametersMessagePubSubType();
   }
}
