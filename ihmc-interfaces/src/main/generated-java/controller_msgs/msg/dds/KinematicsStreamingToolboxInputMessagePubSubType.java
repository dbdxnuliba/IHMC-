package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxInputMessage" defined in "KinematicsStreamingToolboxInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxInputMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsStreamingToolboxInputMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.getCdrSerializedSize(data.getCenterOfMassInput(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyInputs().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.getCdrSerializedSize(data.getRigidBodyInputs().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getControlCenterOfMass());

      controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.write(data.getCenterOfMassInput(), cdr);
      if(data.getRigidBodyInputs().size() <= 20)
      cdr.write_type_e(data.getRigidBodyInputs());else
          throw new RuntimeException("rigid_body_inputs field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setControlCenterOfMass(cdr.read_type_7());
      	
      controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.read(data.getCenterOfMassInput(), cdr);	
      cdr.read_type_e(data.getRigidBodyInputs());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("control_center_of_mass", data.getControlCenterOfMass());
      ser.write_type_a("center_of_mass_input", new controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType(), data.getCenterOfMassInput());

      ser.write_type_e("rigid_body_inputs", data.getRigidBodyInputs());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setControlCenterOfMass(ser.read_type_7("control_center_of_mass"));
      ser.read_type_a("center_of_mass_input", new controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType(), data.getCenterOfMassInput());

      ser.read_type_e("rigid_body_inputs", data.getRigidBodyInputs());
   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxInputMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxInputMessagePubSubType();
   }
}
