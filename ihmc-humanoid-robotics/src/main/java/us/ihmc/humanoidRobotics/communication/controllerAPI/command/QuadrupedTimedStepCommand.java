package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStepCommand implements Command<QuadrupedTimedStepCommand, QuadrupedTimedStepMessage>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private long sequenceId;
   private RobotQuadrant robotQuadrant;
   private FramePoint3D goalPosition = new FramePoint3D();
   private double groundClearance = 0.0;
   private final TimeIntervalCommand timeIntervalCommand = new TimeIntervalCommand();

   public QuadrupedTimedStepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotQuadrant = null;
      groundClearance = 0.0;
      goalPosition.setToNaN();
      timeIntervalCommand.clear();
   }

   @Override
   public void setFromMessage(QuadrupedTimedStepMessage message)
   {
      sequenceId = message.getSequenceId();
      robotQuadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
      groundClearance = message.getGroundClearance();
      goalPosition.setIncludingFrame(worldFrame, message.getGoalPosition());
      timeIntervalCommand.setFromMessage(message.getTimeInterval());
   }

   @Override
   public void set(QuadrupedTimedStepCommand other)
   {
      sequenceId = other.sequenceId;
      robotQuadrant = other.robotQuadrant;
      groundClearance = other.groundClearance;
      goalPosition.setIncludingFrame(other.goalPosition);
      timeIntervalCommand.set(other.timeIntervalCommand);
   }

   public TimeIntervalCommand getTimeIntervalCommand()
   {
      return timeIntervalCommand;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public double getGroundClearance()
   {
      return groundClearance;
   }

   public FramePoint3D getGoalPosition()
   {
      return goalPosition;
   }

   public double getStartTime()
   {
      return timeIntervalCommand.getStartTime();
   }

   public double getEndTime()
   {
      return timeIntervalCommand.getEndTime();
   }

   @Override
   public Class<QuadrupedTimedStepMessage> getMessageClass()
   {
      return QuadrupedTimedStepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotQuadrant != null && timeIntervalCommand.isCommandValid();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
