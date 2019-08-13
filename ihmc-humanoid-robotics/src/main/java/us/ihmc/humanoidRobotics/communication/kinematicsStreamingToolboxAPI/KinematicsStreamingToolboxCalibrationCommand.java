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
   private boolean useGroundHeight = false;
   private boolean useHeadPose = false;
   private boolean useLeftHandPose = false;
   private boolean useRightHandPose = false;

   private double groundHeight = Double.NaN;
   private final Pose3D headPose = new Pose3D();
   private final SideDependentList<Pose3D> handPoses = new SideDependentList<>(side -> new Pose3D());

   public KinematicsStreamingToolboxCalibrationCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      useGroundHeight = false;
      useHeadPose = false;
      useLeftHandPose = false;
      useRightHandPose = false;
      headPose.setToNaN();
      for (RobotSide robotSide : RobotSide.values)
         handPoses.get(robotSide).setToNaN();
   }

   @Override
   public void set(KinematicsStreamingToolboxCalibrationCommand other)
   {
      sequenceId = other.sequenceId;
      useGroundHeight = other.useGroundHeight;
      useHeadPose = other.useHeadPose;
      useLeftHandPose = other.useLeftHandPose;
      useRightHandPose = other.useRightHandPose;

      groundHeight = other.groundHeight;
      headPose.set(other.headPose);
      for (RobotSide robotSide : RobotSide.values)
         handPoses.get(robotSide).set(other.handPoses.get(robotSide));
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxCalibrationMessage message)
   {
      sequenceId = message.getSequenceId();
      useGroundHeight = message.getUseGroundHeight();
      useHeadPose = message.getUseHeadPose();
      useLeftHandPose = message.getUseLeftHandPose();
      useRightHandPose = message.getUseRightHandPose();

      groundHeight = message.getGroundHeight();
      headPose.set(message.getHeadPose());
      handPoses.get(RobotSide.LEFT).set(message.getLeftHandPose());
      handPoses.get(RobotSide.RIGHT).set(message.getRightHandPose());
   }

   public boolean hasGroundHeight()
   {
      return useGroundHeight;
   }

   public boolean hasHeadPose()
   {
      return useHeadPose;
   }

   public boolean hasHandPose(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return useLeftHandPose;
      else
         return useRightHandPose;
   }

   public double getGroundHeight()
   {
      return groundHeight;
   }

   public Pose3D getHeadPose()
   {
      return headPose;
   }

   public Pose3D getHandPose(RobotSide robotSide)
   {
      return handPoses.get(robotSide);
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
