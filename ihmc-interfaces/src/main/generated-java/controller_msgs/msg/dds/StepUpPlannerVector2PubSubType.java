package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerVector2" defined in "StepUpPlannerVector2_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerVector2_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerVector2_.idl instead.
*
*/
public class StepUpPlannerVector2PubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerVector2>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerVector2_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerVector2 data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerVector2 data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerVector2 data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerVector2 data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerVector2 data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getX());

      cdr.write_type_6(data.getY());

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerVector2 data, us.ihmc.idl.CDR cdr)
   {
      data.setX(cdr.read_type_6());
      	
      data.setY(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerVector2 data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("x", data.getX());
      ser.write_type_6("y", data.getY());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerVector2 data)
   {
      data.setX(ser.read_type_6("x"));
      data.setY(ser.read_type_6("y"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerVector2 src, controller_msgs.msg.dds.StepUpPlannerVector2 dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerVector2 createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerVector2();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerVector2 data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerVector2 data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerVector2 src, controller_msgs.msg.dds.StepUpPlannerVector2 dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerVector2PubSubType newInstance()
   {
      return new StepUpPlannerVector2PubSubType();
   }
}
