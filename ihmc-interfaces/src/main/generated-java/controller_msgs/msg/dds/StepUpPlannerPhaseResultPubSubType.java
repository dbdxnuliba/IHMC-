package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerPhaseResult" defined in "StepUpPlannerPhaseResult_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerPhaseResult_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerPhaseResult_.idl instead.
*
*/
public class StepUpPlannerPhaseResultPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerPhaseResult>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerPhaseResult_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerPhaseResult data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerPhaseResult data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getComPosition().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getComPosition().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getComVelocity().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getComVelocity().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getComAcceleration().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getComAcceleration().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeftControls().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getCdrSerializedSize(data.getLeftControls().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRightControls().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StepUpPlannerControlElementPubSubType.getCdrSerializedSize(data.getRightControls().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, us.ihmc.idl.CDR cdr)
   {
      if(data.getComPosition().size() <= 200)
      cdr.write_type_e(data.getComPosition());else
          throw new RuntimeException("com_position field exceeds the maximum length");

      if(data.getComVelocity().size() <= 200)
      cdr.write_type_e(data.getComVelocity());else
          throw new RuntimeException("com_velocity field exceeds the maximum length");

      if(data.getComAcceleration().size() <= 200)
      cdr.write_type_e(data.getComAcceleration());else
          throw new RuntimeException("com_acceleration field exceeds the maximum length");

      if(data.getLeftControls().size() <= 200)
      cdr.write_type_e(data.getLeftControls());else
          throw new RuntimeException("left_controls field exceeds the maximum length");

      if(data.getRightControls().size() <= 200)
      cdr.write_type_e(data.getRightControls());else
          throw new RuntimeException("right_controls field exceeds the maximum length");

      cdr.write_type_6(data.getDuration());

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getComPosition());	
      cdr.read_type_e(data.getComVelocity());	
      cdr.read_type_e(data.getComAcceleration());	
      cdr.read_type_e(data.getLeftControls());	
      cdr.read_type_e(data.getRightControls());	
      data.setDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("com_position", data.getComPosition());
      ser.write_type_e("com_velocity", data.getComVelocity());
      ser.write_type_e("com_acceleration", data.getComAcceleration());
      ser.write_type_e("left_controls", data.getLeftControls());
      ser.write_type_e("right_controls", data.getRightControls());
      ser.write_type_6("duration", data.getDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerPhaseResult data)
   {
      ser.read_type_e("com_position", data.getComPosition());
      ser.read_type_e("com_velocity", data.getComVelocity());
      ser.read_type_e("com_acceleration", data.getComAcceleration());
      ser.read_type_e("left_controls", data.getLeftControls());
      ser.read_type_e("right_controls", data.getRightControls());
      data.setDuration(ser.read_type_6("duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerPhaseResult src, controller_msgs.msg.dds.StepUpPlannerPhaseResult dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerPhaseResult createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerPhaseResult();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerPhaseResult data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerPhaseResult src, controller_msgs.msg.dds.StepUpPlannerPhaseResult dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerPhaseResultPubSubType newInstance()
   {
      return new StepUpPlannerPhaseResultPubSubType();
   }
}
