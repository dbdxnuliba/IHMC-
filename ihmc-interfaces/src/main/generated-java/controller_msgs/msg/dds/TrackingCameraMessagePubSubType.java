package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TrackingCameraMessage" defined in "TrackingCameraMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TrackingCameraMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TrackingCameraMessage_.idl instead.
*
*/
public class TrackingCameraMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.TrackingCameraMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::TrackingCameraMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.TrackingCameraMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.TrackingCameraMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TrackingCameraMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TrackingCameraMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSensorPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getSensorOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TrackingCameraMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_6(data.getQuality());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getSensorPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getSensorOrientation(), cdr);
   }

   public static void read(controller_msgs.msg.dds.TrackingCameraMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setQuality(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getSensorPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getSensorOrientation(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TrackingCameraMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_6("quality", data.getQuality());
      ser.write_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.write_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TrackingCameraMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setQuality(ser.read_type_6("quality"));
      ser.read_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.read_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

   }

   public static void staticCopy(controller_msgs.msg.dds.TrackingCameraMessage src, controller_msgs.msg.dds.TrackingCameraMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.TrackingCameraMessage createData()
   {
      return new controller_msgs.msg.dds.TrackingCameraMessage();
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
   
   public void serialize(controller_msgs.msg.dds.TrackingCameraMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.TrackingCameraMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.TrackingCameraMessage src, controller_msgs.msg.dds.TrackingCameraMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TrackingCameraMessagePubSubType newInstance()
   {
      return new TrackingCameraMessagePubSubType();
   }
}
