package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "State6d" defined in "State6d_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from State6d_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit State6d_.idl instead.
*
*/
public class State6dPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.State6d>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::State6d_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.State6d data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.State6d data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.TwistPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.AccelPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.State6d data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.State6d data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TwistPubSubType.getCdrSerializedSize(data.getTwist(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.AccelPubSubType.getCdrSerializedSize(data.getAccel(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.State6d data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
      geometry_msgs.msg.dds.TwistPubSubType.write(data.getTwist(), cdr);
      geometry_msgs.msg.dds.AccelPubSubType.write(data.getAccel(), cdr);
   }

   public static void read(controller_msgs.msg.dds.State6d data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	
      geometry_msgs.msg.dds.TwistPubSubType.read(data.getTwist(), cdr);	
      geometry_msgs.msg.dds.AccelPubSubType.read(data.getAccel(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.State6d data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.write_type_a("twist", new geometry_msgs.msg.dds.TwistPubSubType(), data.getTwist());

      ser.write_type_a("accel", new geometry_msgs.msg.dds.AccelPubSubType(), data.getAccel());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.State6d data)
   {
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.read_type_a("twist", new geometry_msgs.msg.dds.TwistPubSubType(), data.getTwist());

      ser.read_type_a("accel", new geometry_msgs.msg.dds.AccelPubSubType(), data.getAccel());

   }

   public static void staticCopy(controller_msgs.msg.dds.State6d src, controller_msgs.msg.dds.State6d dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.State6d createData()
   {
      return new controller_msgs.msg.dds.State6d();
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
   
   public void serialize(controller_msgs.msg.dds.State6d data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.State6d data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.State6d src, controller_msgs.msg.dds.State6d dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public State6dPubSubType newInstance()
   {
      return new State6dPubSubType();
   }
}
