package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedStepMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class QuadrupedStepCommand implements Command<QuadrupedStepCommand, QuadrupedStepMessage>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private long sequenceId;
   private RobotQuadrant robotQuadrant;
   private FramePoint3D goalPosition = new FramePoint3D();
   private double groundClearance = 0.0;
   private TrajectoryType trajectoryType;
   private final RecyclingArrayList<Point3D> customPositionWaypoints = new RecyclingArrayList<Point3D>(2, Point3D.class);

   public QuadrupedStepCommand()
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
      customPositionWaypoints.clear();
   }

   @Override
   public void setFromMessage(QuadrupedStepMessage message)
   {
      sequenceId = message.getSequenceId();
      robotQuadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
      groundClearance = message.getGroundClearance();
      goalPosition.setIncludingFrame(worldFrame, message.getGoalPosition());
      trajectoryType = TrajectoryType.fromByte(message.getTrajectoryType());

      customPositionWaypoints.clear();      
      for(int i = 0; i < message.getCustomPositionWaypoints().size(); i++)
      {
         customPositionWaypoints.add().set(message.getCustomPositionWaypoints().get(i));
      }
   }

   @Override
   public void set(QuadrupedStepCommand other)
   {
      sequenceId = other.sequenceId;
      robotQuadrant = other.robotQuadrant;
      groundClearance = other.groundClearance;
      goalPosition.setIncludingFrame(other.goalPosition);
      trajectoryType = other.getTrajectoryType();
      
      customPositionWaypoints.clear();      
      for(int i = 0; i < other.getCustomPositionWaypoints().size(); i++)
      {
         customPositionWaypoints.add().set(other.getCustomPositionWaypoints().get(i));
      }
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
   
   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }
   
   public RecyclingArrayList<Point3D> getCustomPositionWaypoints()
   {
      return customPositionWaypoints;
   }

   @Override
   public Class<QuadrupedStepMessage> getMessageClass()
   {
      return QuadrupedStepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotQuadrant != null;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}