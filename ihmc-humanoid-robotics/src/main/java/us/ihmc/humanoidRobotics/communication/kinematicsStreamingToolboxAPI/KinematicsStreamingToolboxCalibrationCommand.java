package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxCalibrationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KinematicsStreamingToolboxCalibrationCommand
      implements Command<KinematicsStreamingToolboxCalibrationCommand, KinematicsStreamingToolboxCalibrationMessage>
{
   private long sequenceId;
   private final Pose3D headPose = new Pose3D();
   private final SideDependentList<Pose3D> handPoses = new SideDependentList<>(side -> new Pose3D());

   public KinematicsStreamingToolboxCalibrationCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      headPose.setToNaN();
      for (RobotSide robotSide : RobotSide.values)
         handPoses.get(robotSide).setToNaN();
   }

   @Override
   public void set(KinematicsStreamingToolboxCalibrationCommand other)
   {
      sequenceId = other.sequenceId;

      headPose.set(other.headPose);
      for (RobotSide robotSide : RobotSide.values)
         handPoses.get(robotSide).set(other.handPoses.get(robotSide));
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxCalibrationMessage message)
   {
      sequenceId = message.getSequenceId();

      headPose.set(message.getHeadPose());
      handPoses.get(RobotSide.LEFT).set(message.getLeftHandPose());
      handPoses.get(RobotSide.RIGHT).set(message.getRightHandPose());
   }

   public Pose3D getHeadPose()
   {
      return headPose;
   }

   public Pose3D getHandPose(RobotSide robotSide)
   {
      return handPoses.get(robotSide);
   }

   public SideDependentList<Pose3D> getHandPoses()
   {
      return handPoses;
   }

   @Override
   public Class<KinematicsStreamingToolboxCalibrationMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxCalibrationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
