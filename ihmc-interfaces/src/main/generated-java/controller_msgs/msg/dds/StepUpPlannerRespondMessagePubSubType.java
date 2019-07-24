package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerRespondMessage" defined in "StepUpPlannerRespondMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerRespondMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerRespondMessage_.idl instead.
*
*/
public class StepUpPlannerRespondMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerRespondMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerRespondMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerRespondMessage data) throws java.io.IOException
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
          current_alignment += controller_msgs.msg.dds.StepUpPlannerPhaseResultPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CenterOfMassTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerRespondMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPhasesResult().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerPhaseResultPubSubType.getCdrSerializedSize(data.getPhasesResult().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getComMessages().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CenterOfMassTrajectoryMessagePubSubType.getCdrSerializedSize(data.getComMessages().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFoostepMessages().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFoostepMessages().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPhasesResult().size() <= 20)
      cdr.write_type_e(data.getPhasesResult());else
          throw new RuntimeException("phases_result field exceeds the maximum length");

      cdr.write_type_6(data.getTotalDuration());

      if(data.getComMessages().size() <= 20)
      cdr.write_type_e(data.getComMessages());else
          throw new RuntimeException("com_messages field exceeds the maximum length");

      if(data.getFoostepMessages().size() <= 20)
      cdr.write_type_e(data.getFoostepMessages());else
          throw new RuntimeException("foostep_messages field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPhasesResult());	
      data.setTotalDuration(cdr.read_type_6());
      	
      cdr.read_type_e(data.getComMessages());	
      cdr.read_type_e(data.getFoostepMessages());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("phases_result", data.getPhasesResult());
      ser.write_type_6("total_duration", data.getTotalDuration());
      ser.write_type_e("com_messages", data.getComMessages());
      ser.write_type_e("foostep_messages", data.getFoostepMessages());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerRespondMessage data)
   {
      ser.read_type_e("phases_result", data.getPhasesResult());
      data.setTotalDuration(ser.read_type_6("total_duration"));
      ser.read_type_e("com_messages", data.getComMessages());
      ser.read_type_e("foostep_messages", data.getFoostepMessages());
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerRespondMessage src, controller_msgs.msg.dds.StepUpPlannerRespondMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerRespondMessage createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerRespondMessage();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerRespondMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerRespondMessage src, controller_msgs.msg.dds.StepUpPlannerRespondMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerRespondMessagePubSubType newInstance()
   {
      return new StepUpPlannerRespondMessagePubSubType();
   }
}
