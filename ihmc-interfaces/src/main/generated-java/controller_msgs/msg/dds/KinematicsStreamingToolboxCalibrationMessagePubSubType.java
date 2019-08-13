package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxCalibrationMessage" defined in "KinematicsStreamingToolboxCalibrationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxCalibrationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxCalibrationMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxCalibrationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsStreamingToolboxCalibrationMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getHeadPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getLeftHandPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getRightHandPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getUseGroundHeight());

      cdr.write_type_7(data.getUseHeadPose());

      cdr.write_type_7(data.getUseLeftHandPose());

      cdr.write_type_7(data.getUseRightHandPose());

      cdr.write_type_6(data.getGroundHeight());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getHeadPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getLeftHandPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getRightHandPose(), cdr);
   }

   public static void read(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setUseGroundHeight(cdr.read_type_7());
      	
      data.setUseHeadPose(cdr.read_type_7());
      	
      data.setUseLeftHandPose(cdr.read_type_7());
      	
      data.setUseRightHandPose(cdr.read_type_7());
      	
      data.setGroundHeight(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getHeadPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getLeftHandPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getRightHandPose(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("use_ground_height", data.getUseGroundHeight());
      ser.write_type_7("use_head_pose", data.getUseHeadPose());
      ser.write_type_7("use_left_hand_pose", data.getUseLeftHandPose());
      ser.write_type_7("use_right_hand_pose", data.getUseRightHandPose());
      ser.write_type_6("ground_height", data.getGroundHeight());
      ser.write_type_a("head_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getHeadPose());

      ser.write_type_a("left_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftHandPose());

      ser.write_type_a("right_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightHandPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setUseGroundHeight(ser.read_type_7("use_ground_height"));
      data.setUseHeadPose(ser.read_type_7("use_head_pose"));
      data.setUseLeftHandPose(ser.read_type_7("use_left_hand_pose"));
      data.setUseRightHandPose(ser.read_type_7("use_right_hand_pose"));
      data.setGroundHeight(ser.read_type_6("ground_height"));
      ser.read_type_a("head_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getHeadPose());

      ser.read_type_a("left_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftHandPose());

      ser.read_type_a("right_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightHandPose());

   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxCalibrationMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxCalibrationMessagePubSubType();
   }
}
