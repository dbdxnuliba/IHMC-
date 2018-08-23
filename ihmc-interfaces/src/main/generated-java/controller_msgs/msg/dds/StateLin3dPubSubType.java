package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StateLin3d" defined in "StateLin3d_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StateLin3d_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StateLin3d_.idl instead.
*
*/
public class StateLin3dPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StateLin3d>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StateLin3d_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StateLin3d data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StateLin3d data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StateLin3d data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StateLin3d data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPos(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getVel(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAcc(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StateLin3d data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.write(data.getPos(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getVel(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAcc(), cdr);
   }

   public static void read(controller_msgs.msg.dds.StateLin3d data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPos(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getVel(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAcc(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StateLin3d data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("pos", new geometry_msgs.msg.dds.PointPubSubType(), data.getPos());

      ser.write_type_a("vel", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getVel());

      ser.write_type_a("acc", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAcc());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StateLin3d data)
   {
      ser.read_type_a("pos", new geometry_msgs.msg.dds.PointPubSubType(), data.getPos());

      ser.read_type_a("vel", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getVel());

      ser.read_type_a("acc", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAcc());

   }

   public static void staticCopy(controller_msgs.msg.dds.StateLin3d src, controller_msgs.msg.dds.StateLin3d dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StateLin3d createData()
   {
      return new controller_msgs.msg.dds.StateLin3d();
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
   
   public void serialize(controller_msgs.msg.dds.StateLin3d data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StateLin3d data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StateLin3d src, controller_msgs.msg.dds.StateLin3d dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StateLin3dPubSubType newInstance()
   {
      return new StateLin3dPubSubType();
   }
}
