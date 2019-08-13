package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxCalibrationStatusMessage" defined in "KinematicsStreamingToolboxCalibrationStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxCalibrationStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxCalibrationStatusMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxCalibrationStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsStreamingToolboxCalibrationStatusMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCurrentStatus().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getCurrentStatus().length() <= 255)
      cdr.write_type_d(data.getCurrentStatus());else
          throw new RuntimeException("current_status field exceeds the maximum length");

      cdr.write_type_7(data.getIsCalibrationDone());

   }

   public static void read(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getCurrentStatus());	
      data.setIsCalibrationDone(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("current_status", data.getCurrentStatus());
      ser.write_type_7("is_calibration_done", data.getIsCalibrationDone());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("current_status", data.getCurrentStatus());
      data.setIsCalibrationDone(ser.read_type_7("is_calibration_done"));
   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxCalibrationStatusMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxCalibrationStatusMessagePubSubType();
   }
}
