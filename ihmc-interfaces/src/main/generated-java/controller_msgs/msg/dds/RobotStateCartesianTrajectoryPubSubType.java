package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotStateCartesianTrajectory" defined in "RobotStateCartesianTrajectory_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotStateCartesianTrajectory_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotStateCartesianTrajectory_.idl instead.
*
*/
public class RobotStateCartesianTrajectoryPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RobotStateCartesianTrajectory>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RobotStateCartesianTrajectory_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RobotStateCartesianTrajectory data) throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.RobotStateCartesianPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotStateCartesianTrajectory data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPoints().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.RobotStateCartesianPubSubType.getCdrSerializedSize(data.getPoints().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getPoints().size() <= 100)
      cdr.write_type_e(data.getPoints());else
          throw new RuntimeException("points field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getPoints());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("points", data.getPoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RobotStateCartesianTrajectory data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("points", data.getPoints());
   }

   public static void staticCopy(controller_msgs.msg.dds.RobotStateCartesianTrajectory src, controller_msgs.msg.dds.RobotStateCartesianTrajectory dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RobotStateCartesianTrajectory createData()
   {
      return new controller_msgs.msg.dds.RobotStateCartesianTrajectory();
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
   
   public void serialize(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RobotStateCartesianTrajectory data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RobotStateCartesianTrajectory src, controller_msgs.msg.dds.RobotStateCartesianTrajectory dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotStateCartesianTrajectoryPubSubType newInstance()
   {
      return new RobotStateCartesianTrajectoryPubSubType();
   }
}
