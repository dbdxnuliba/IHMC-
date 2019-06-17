package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CollisionManagerMessage" defined in "CollisionManagerMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CollisionManagerMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CollisionManagerMessage_.idl instead.
*
*/
public class CollisionManagerMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CollisionManagerMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CollisionManagerMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CollisionManagerMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CollisionManagerMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CollisionManagerMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CollisionManagerMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPlanarRegionsList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CollisionManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPlanarRegionsList().size() <= 100)
      cdr.write_type_e(data.getPlanarRegionsList());else
          throw new RuntimeException("planar_regions_list field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.CollisionManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPlanarRegionsList());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CollisionManagerMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("planar_regions_list", data.getPlanarRegionsList());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CollisionManagerMessage data)
   {
      ser.read_type_e("planar_regions_list", data.getPlanarRegionsList());
   }

   public static void staticCopy(controller_msgs.msg.dds.CollisionManagerMessage src, controller_msgs.msg.dds.CollisionManagerMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CollisionManagerMessage createData()
   {
      return new controller_msgs.msg.dds.CollisionManagerMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CollisionManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CollisionManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CollisionManagerMessage src, controller_msgs.msg.dds.CollisionManagerMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CollisionManagerMessagePubSubType newInstance()
   {
      return new CollisionManagerMessagePubSubType();
   }
}
