package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerStepParameters" defined in "StepUpPlannerStepParameters_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerStepParameters_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerStepParameters_.idl instead.
*
*/
public class StepUpPlannerStepParametersPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerStepParameters>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerStepParameters_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerStepParameters data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerStepParameters data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerStepParameters data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerStepParameters data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootVertices().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.getCdrSerializedSize(data.getFootVertices().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.getCdrSerializedSize(data.getCenterOffset(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerStepParameters data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getState());

      if(data.getFootVertices().size() <= 10)
      cdr.write_type_e(data.getFootVertices());else
          throw new RuntimeException("foot_vertices field exceeds the maximum length");

      cdr.write_type_6(data.getScale());

      controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.write(data.getCenterOffset(), cdr);
   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerStepParameters data, us.ihmc.idl.CDR cdr)
   {
      data.setState(cdr.read_type_9());
      	
      cdr.read_type_e(data.getFootVertices());	
      data.setScale(cdr.read_type_6());
      	
      controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.read(data.getCenterOffset(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerStepParameters data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("state", data.getState());
      ser.write_type_e("foot_vertices", data.getFootVertices());
      ser.write_type_6("scale", data.getScale());
      ser.write_type_a("center_offset", new controller_msgs.msg.dds.StepUpPlannerVector2PubSubType(), data.getCenterOffset());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerStepParameters data)
   {
      data.setState(ser.read_type_9("state"));
      ser.read_type_e("foot_vertices", data.getFootVertices());
      data.setScale(ser.read_type_6("scale"));
      ser.read_type_a("center_offset", new controller_msgs.msg.dds.StepUpPlannerVector2PubSubType(), data.getCenterOffset());

   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerStepParameters src, controller_msgs.msg.dds.StepUpPlannerStepParameters dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerStepParameters createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerStepParameters();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerStepParameters data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerStepParameters data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerStepParameters src, controller_msgs.msg.dds.StepUpPlannerStepParameters dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerStepParametersPubSubType newInstance()
   {
      return new StepUpPlannerStepParametersPubSubType();
   }
}
