package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CollisionAvoidanceManagerMessage" defined in "CollisionAvoidanceManagerMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CollisionAvoidanceManagerMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CollisionAvoidanceManagerMessage_.idl instead.
*
*/
public class CollisionAvoidanceManagerMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CollisionAvoidanceManagerMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CollisionAvoidanceManagerMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data) throws java.io.IOException
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
          current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPlanarRegionsList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPlanarRegionsList().size() <= 20)
      cdr.write_type_e(data.getPlanarRegionsList());else
          throw new RuntimeException("planar_regions_list field exceeds the maximum length");

      cdr.write_type_7(data.getConsiderOnlyEdges());

      cdr.write_type_9(data.getMode());

      cdr.write_type_4(data.getSequenceId());

   }

   public static void read(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPlanarRegionsList());	
      data.setConsiderOnlyEdges(cdr.read_type_7());
      	
      data.setMode(cdr.read_type_9());
      	
      data.setSequenceId(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("planar_regions_list", data.getPlanarRegionsList());
      ser.write_type_7("consider_only_edges", data.getConsiderOnlyEdges());
      ser.write_type_9("mode", data.getMode());
      ser.write_type_4("sequence_id", data.getSequenceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data)
   {
      ser.read_type_e("planar_regions_list", data.getPlanarRegionsList());
      data.setConsiderOnlyEdges(ser.read_type_7("consider_only_edges"));
      data.setMode(ser.read_type_9("mode"));
      data.setSequenceId(ser.read_type_4("sequence_id"));
   }

   public static void staticCopy(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage src, controller_msgs.msg.dds.CollisionAvoidanceManagerMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CollisionAvoidanceManagerMessage createData()
   {
      return new controller_msgs.msg.dds.CollisionAvoidanceManagerMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CollisionAvoidanceManagerMessage src, controller_msgs.msg.dds.CollisionAvoidanceManagerMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CollisionAvoidanceManagerMessagePubSubType newInstance()
   {
      return new CollisionAvoidanceManagerMessagePubSubType();
   }
}
