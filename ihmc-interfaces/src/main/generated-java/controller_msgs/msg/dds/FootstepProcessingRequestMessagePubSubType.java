package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepProcessingRequestMessage" defined in "FootstepProcessingRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepProcessingRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepProcessingRequestMessage_.idl instead.
*
*/
public class FootstepProcessingRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepProcessingRequestMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepProcessingRequestMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepProcessingRequestMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepProcessingRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootstepDataListMessage(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLeftFootLocation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getLeftFootOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRightFootLocation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRightFootOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootstepDataListMessage(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getLeftFootLocation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getLeftFootOrientation(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getRightFootLocation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRightFootOrientation(), cdr);
   }

   public static void read(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootstepDataListMessage(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLeftFootLocation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getLeftFootOrientation(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRightFootLocation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRightFootOrientation(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("footstep_data_list_message", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataListMessage());

      ser.write_type_a("left_foot_location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLeftFootLocation());

      ser.write_type_a("left_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLeftFootOrientation());

      ser.write_type_a("right_foot_location", new geometry_msgs.msg.dds.PointPubSubType(), data.getRightFootLocation());

      ser.write_type_a("right_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRightFootOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepProcessingRequestMessage data)
   {
      ser.read_type_a("footstep_data_list_message", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataListMessage());

      ser.read_type_a("left_foot_location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLeftFootLocation());

      ser.read_type_a("left_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLeftFootOrientation());

      ser.read_type_a("right_foot_location", new geometry_msgs.msg.dds.PointPubSubType(), data.getRightFootLocation());

      ser.read_type_a("right_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRightFootOrientation());

   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepProcessingRequestMessage src, controller_msgs.msg.dds.FootstepProcessingRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepProcessingRequestMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepProcessingRequestMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepProcessingRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepProcessingRequestMessage src, controller_msgs.msg.dds.FootstepProcessingRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepProcessingRequestMessagePubSubType newInstance()
   {
      return new FootstepProcessingRequestMessagePubSubType();
   }
}
