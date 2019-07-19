package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerPhase" defined in "StepUpPlannerPhase_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerPhase_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerPhase_.idl instead.
*
*/
public class StepUpPlannerPhasePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerPhase>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerPhase_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerPhase data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerPhase data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerPhase data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerPhase data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getLeftFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getRightFootPose(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerPhase data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.write(data.getLeftFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getRightFootPose(), cdr);
      cdr.write_type_6(data.getDesiredDuration());

      cdr.write_type_6(data.getMinimumDuration());

      cdr.write_type_6(data.getMaximumDuration());

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerPhase data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.read(data.getLeftFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getRightFootPose(), cdr);	
      data.setDesiredDuration(cdr.read_type_6());
      	
      data.setMinimumDuration(cdr.read_type_6());
      	
      data.setMaximumDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerPhase data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftFootPose());

      ser.write_type_a("right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightFootPose());

      ser.write_type_6("desired_duration", data.getDesiredDuration());
      ser.write_type_6("minimum_duration", data.getMinimumDuration());
      ser.write_type_6("maximum_duration", data.getMaximumDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerPhase data)
   {
      ser.read_type_a("left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftFootPose());

      ser.read_type_a("right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightFootPose());

      data.setDesiredDuration(ser.read_type_6("desired_duration"));
      data.setMinimumDuration(ser.read_type_6("minimum_duration"));
      data.setMaximumDuration(ser.read_type_6("maximum_duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerPhase src, controller_msgs.msg.dds.StepUpPlannerPhase dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerPhase createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerPhase();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerPhase data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerPhase data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerPhase src, controller_msgs.msg.dds.StepUpPlannerPhase dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerPhasePubSubType newInstance()
   {
      return new StepUpPlannerPhasePubSubType();
   }
}
