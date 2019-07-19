package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerRequestMessage" defined in "StepUpPlannerRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerRequestMessage_.idl instead.
*
*/
public class StepUpPlannerRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerRequestMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerRequestMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerRequestMessage data) throws java.io.IOException
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
          current_alignment += controller_msgs.msg.dds.StepUpPlannerPhasePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPhases().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerPhasePubSubType.getCdrSerializedSize(data.getPhases().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredComPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredComVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getInitialComPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getInitialComVelocity(), current_alignment);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getCdrSerializedSize(data.getLeftDesiredFinalControl(), current_alignment);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getCdrSerializedSize(data.getRightDesiredFinalControl(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPhases().size() <= 20)
      cdr.write_type_e(data.getPhases());else
          throw new RuntimeException("phases field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredComPosition(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredComVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getInitialComPosition(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getInitialComVelocity(), cdr);
      controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.write(data.getLeftDesiredFinalControl(), cdr);
      controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.write(data.getRightDesiredFinalControl(), cdr);
      cdr.write_type_6(data.getDesiredLegLength());

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPhases());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredComPosition(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredComVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getInitialComPosition(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getInitialComVelocity(), cdr);	
      controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.read(data.getLeftDesiredFinalControl(), cdr);	
      controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.read(data.getRightDesiredFinalControl(), cdr);	
      data.setDesiredLegLength(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("phases", data.getPhases());
      ser.write_type_a("desired_com_position", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredComPosition());

      ser.write_type_a("desired_com_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredComVelocity());

      ser.write_type_a("initial_com_position", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getInitialComPosition());

      ser.write_type_a("initial_com_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getInitialComVelocity());

      ser.write_type_a("left_desired_final_control", new controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType(), data.getLeftDesiredFinalControl());

      ser.write_type_a("right_desired_final_control", new controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType(), data.getRightDesiredFinalControl());

      ser.write_type_6("desired_leg_length", data.getDesiredLegLength());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerRequestMessage data)
   {
      ser.read_type_e("phases", data.getPhases());
      ser.read_type_a("desired_com_position", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredComPosition());

      ser.read_type_a("desired_com_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredComVelocity());

      ser.read_type_a("initial_com_position", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getInitialComPosition());

      ser.read_type_a("initial_com_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getInitialComVelocity());

      ser.read_type_a("left_desired_final_control", new controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType(), data.getLeftDesiredFinalControl());

      ser.read_type_a("right_desired_final_control", new controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType(), data.getRightDesiredFinalControl());

      data.setDesiredLegLength(ser.read_type_6("desired_leg_length"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerRequestMessage src, controller_msgs.msg.dds.StepUpPlannerRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerRequestMessage createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerRequestMessage();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerRequestMessage src, controller_msgs.msg.dds.StepUpPlannerRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerRequestMessagePubSubType newInstance()
   {
      return new StepUpPlannerRequestMessagePubSubType();
   }
}
