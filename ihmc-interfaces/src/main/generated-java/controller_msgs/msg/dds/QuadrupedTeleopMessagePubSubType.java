package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedTeleopMessage" defined in "QuadrupedTeleopMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedTeleopMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedTeleopMessage_.idl instead.
*
*/
public class QuadrupedTeleopMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedTeleopMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedTeleopMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedTeleopMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedTeleopMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedTeleopMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedTeleopMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredVelocity(), current_alignment);

      current_alignment += controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType.getCdrSerializedSize(data.getXGaitSettings(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedTeleopMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getRequestWalk());

      cdr.write_type_7(data.getAreStepsAdjustable());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredVelocity(), cdr);
      controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType.write(data.getXGaitSettings(), cdr);
   }

   public static void read(controller_msgs.msg.dds.QuadrupedTeleopMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRequestWalk(cdr.read_type_7());
      	
      data.setAreStepsAdjustable(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredVelocity(), cdr);	
      controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType.read(data.getXGaitSettings(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedTeleopMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("request_walk", data.getRequestWalk());
      ser.write_type_7("are_steps_adjustable", data.getAreStepsAdjustable());
      ser.write_type_a("desired_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredVelocity());

      ser.write_type_a("x_gait_settings", new controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType(), data.getXGaitSettings());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedTeleopMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRequestWalk(ser.read_type_7("request_walk"));
      data.setAreStepsAdjustable(ser.read_type_7("are_steps_adjustable"));
      ser.read_type_a("desired_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredVelocity());

      ser.read_type_a("x_gait_settings", new controller_msgs.msg.dds.QuadrupedXGaitSettingsPacketPubSubType(), data.getXGaitSettings());

   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedTeleopMessage src, controller_msgs.msg.dds.QuadrupedTeleopMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedTeleopMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedTeleopMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedTeleopMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedTeleopMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedTeleopMessage src, controller_msgs.msg.dds.QuadrupedTeleopMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedTeleopMessagePubSubType newInstance()
   {
      return new QuadrupedTeleopMessagePubSubType();
   }
}
