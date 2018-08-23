package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotStateCartesian" defined in "RobotStateCartesian_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotStateCartesian_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotStateCartesian_.idl instead.
*
*/
public class RobotStateCartesianPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RobotStateCartesian>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RobotStateCartesian_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RobotStateCartesian data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RobotStateCartesian data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.DurationPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.State6dPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StateLin3dPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotStateCartesian data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotStateCartesian data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.DurationPubSubType.getCdrSerializedSize(data.getTimeFromStart(), current_alignment);

      current_alignment += controller_msgs.msg.dds.State6dPubSubType.getCdrSerializedSize(data.getBase(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEeMotion().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.StateLin3dPubSubType.getCdrSerializedSize(data.getEeMotion().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEeForces().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getEeForces().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getEeContact().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RobotStateCartesian data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.DurationPubSubType.write(data.getTimeFromStart(), cdr);
      controller_msgs.msg.dds.State6dPubSubType.write(data.getBase(), cdr);
      if(data.getEeMotion().size() <= 100)
      cdr.write_type_e(data.getEeMotion());else
          throw new RuntimeException("ee_motion field exceeds the maximum length");

      if(data.getEeForces().size() <= 100)
      cdr.write_type_e(data.getEeForces());else
          throw new RuntimeException("ee_forces field exceeds the maximum length");

      if(data.getEeContact().size() <= 100)
      cdr.write_type_e(data.getEeContact());else
          throw new RuntimeException("ee_contact field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.RobotStateCartesian data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.DurationPubSubType.read(data.getTimeFromStart(), cdr);	
      controller_msgs.msg.dds.State6dPubSubType.read(data.getBase(), cdr);	
      cdr.read_type_e(data.getEeMotion());	
      cdr.read_type_e(data.getEeForces());	
      cdr.read_type_e(data.getEeContact());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RobotStateCartesian data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("time_from_start", new controller_msgs.msg.dds.DurationPubSubType(), data.getTimeFromStart());

      ser.write_type_a("base", new controller_msgs.msg.dds.State6dPubSubType(), data.getBase());

      ser.write_type_e("ee_motion", data.getEeMotion());
      ser.write_type_e("ee_forces", data.getEeForces());
      ser.write_type_e("ee_contact", data.getEeContact());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RobotStateCartesian data)
   {
      ser.read_type_a("time_from_start", new controller_msgs.msg.dds.DurationPubSubType(), data.getTimeFromStart());

      ser.read_type_a("base", new controller_msgs.msg.dds.State6dPubSubType(), data.getBase());

      ser.read_type_e("ee_motion", data.getEeMotion());
      ser.read_type_e("ee_forces", data.getEeForces());
      ser.read_type_e("ee_contact", data.getEeContact());
   }

   public static void staticCopy(controller_msgs.msg.dds.RobotStateCartesian src, controller_msgs.msg.dds.RobotStateCartesian dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RobotStateCartesian createData()
   {
      return new controller_msgs.msg.dds.RobotStateCartesian();
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
   
   public void serialize(controller_msgs.msg.dds.RobotStateCartesian data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RobotStateCartesian data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RobotStateCartesian src, controller_msgs.msg.dds.RobotStateCartesian dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotStateCartesianPubSubType newInstance()
   {
      return new RobotStateCartesianPubSubType();
   }
}
