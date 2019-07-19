package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerPhaseParameters" defined in "StepUpPlannerPhaseParameters_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerPhaseParameters_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerPhaseParameters_.idl instead.
*
*/
public class StepUpPlannerPhaseParametersPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerPhaseParameters>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerPhaseParameters_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerPhaseParameters data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.getCdrSerializedSize(data.getLeftStepParameters(), current_alignment);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.getCdrSerializedSize(data.getRightStepParameters(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.write(data.getLeftStepParameters(), cdr);
      controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.write(data.getRightStepParameters(), cdr);
   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.read(data.getLeftStepParameters(), cdr);	
      controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType.read(data.getRightStepParameters(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("left_step_parameters", new controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType(), data.getLeftStepParameters());

      ser.write_type_a("right_step_parameters", new controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType(), data.getRightStepParameters());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerPhaseParameters data)
   {
      ser.read_type_a("left_step_parameters", new controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType(), data.getLeftStepParameters());

      ser.read_type_a("right_step_parameters", new controller_msgs.msg.dds.StepUpPlannerStepParametersPubSubType(), data.getRightStepParameters());

   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerPhaseParameters src, controller_msgs.msg.dds.StepUpPlannerPhaseParameters dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerPhaseParameters createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerPhaseParameters();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerPhaseParameters data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerPhaseParameters src, controller_msgs.msg.dds.StepUpPlannerPhaseParameters dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerPhaseParametersPubSubType newInstance()
   {
      return new StepUpPlannerPhaseParametersPubSubType();
   }
}
