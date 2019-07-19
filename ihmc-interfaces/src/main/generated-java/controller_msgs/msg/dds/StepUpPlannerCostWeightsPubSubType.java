package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerCostWeights" defined in "StepUpPlannerCostWeights_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerCostWeights_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerCostWeights_.idl instead.
*
*/
public class StepUpPlannerCostWeightsPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerCostWeights>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerCostWeights_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerCostWeights data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerCostWeights data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerCostWeights data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerCostWeights data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerCostWeights data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getCop());

      cdr.write_type_6(data.getTorques());

      cdr.write_type_6(data.getControlMultipliers());

      cdr.write_type_6(data.getFinalControl());

      cdr.write_type_6(data.getMaxControlMultiplier());

      cdr.write_type_6(data.getFinalState());

      cdr.write_type_6(data.getControlVariations());

      cdr.write_type_6(data.getDurationsDifference());

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerCostWeights data, us.ihmc.idl.CDR cdr)
   {
      data.setCop(cdr.read_type_6());
      	
      data.setTorques(cdr.read_type_6());
      	
      data.setControlMultipliers(cdr.read_type_6());
      	
      data.setFinalControl(cdr.read_type_6());
      	
      data.setMaxControlMultiplier(cdr.read_type_6());
      	
      data.setFinalState(cdr.read_type_6());
      	
      data.setControlVariations(cdr.read_type_6());
      	
      data.setDurationsDifference(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerCostWeights data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("cop", data.getCop());
      ser.write_type_6("torques", data.getTorques());
      ser.write_type_6("control_multipliers", data.getControlMultipliers());
      ser.write_type_6("final_control", data.getFinalControl());
      ser.write_type_6("max_control_multiplier", data.getMaxControlMultiplier());
      ser.write_type_6("final_state", data.getFinalState());
      ser.write_type_6("control_variations", data.getControlVariations());
      ser.write_type_6("durations_difference", data.getDurationsDifference());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerCostWeights data)
   {
      data.setCop(ser.read_type_6("cop"));
      data.setTorques(ser.read_type_6("torques"));
      data.setControlMultipliers(ser.read_type_6("control_multipliers"));
      data.setFinalControl(ser.read_type_6("final_control"));
      data.setMaxControlMultiplier(ser.read_type_6("max_control_multiplier"));
      data.setFinalState(ser.read_type_6("final_state"));
      data.setControlVariations(ser.read_type_6("control_variations"));
      data.setDurationsDifference(ser.read_type_6("durations_difference"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerCostWeights src, controller_msgs.msg.dds.StepUpPlannerCostWeights dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerCostWeights createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerCostWeights();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerCostWeights data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerCostWeights data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerCostWeights src, controller_msgs.msg.dds.StepUpPlannerCostWeights dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerCostWeightsPubSubType newInstance()
   {
      return new StepUpPlannerCostWeightsPubSubType();
   }
}
