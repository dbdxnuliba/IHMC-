package us.ihmc.quadrupedBasics.gait;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TimeIntervalCommand;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public class QuadrupedTimedStep implements TimeIntervalProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_RIGHT;
   private Point3D goalPosition = new Point3D(0.0, 0.0, 0.0);
   private double groundClearance = 0.0;
   private final TimeInterval timeInterval = new TimeInterval();

   public QuadrupedTimedStep()
   {
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, Point3DBasics goalPosition, double groundClearance, TimeIntervalBasics timeInterval)
   {
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
      setTimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(QuadrupedTimedStep other)
   {
      this(other.getRobotQuadrant(), other.getGoalPositionInternal(), other.getGroundClearance(), other.getTimeInterval());
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   protected Point3DBasics getGoalPositionInternal()
   {
      return goalPosition;
   }

   public Point3DReadOnly getGoalPosition()
   {
      return goalPosition;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return worldFrame;
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }

   public void setGoalPosition(Point3DReadOnly goalPosition)
   {
      getGoalPositionInternal().set(goalPosition);
   }

   public void setGoalPosition(FramePoint3D goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(getReferenceFrame());
      getGoalPositionInternal().set(goalPosition);
      goalPosition.changeFrame(originalFrame);
   }

   public double getGroundClearance()
   {
      return groundClearance;
   }

   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance = groundClearance;
   }

   public void setTimeInterval(TimeIntervalBasics timeInterval)
   {
      getTimeInterval().set(timeInterval);
   }

   public void setTimeInterval(TimeIntervalCommand command)
   {
      command.getTimeInterval(getTimeInterval());
   }

   public void set(QuadrupedTimedStep other)
   {
      setRobotQuadrant(other.getRobotQuadrant());
      setGoalPosition(other.getGoalPosition());
      setGroundClearance(other.getGroundClearance());
      setTimeInterval(other.getTimeInterval());
   }

   public void set(QuadrupedTimedStepCommand command)
   {
      setRobotQuadrant(command.getRobotQuadrant());
      setGoalPosition(command.getGoalPosition());
      setGroundClearance(command.getGroundClearance());
      setTimeInterval(command.getTimeIntervalCommand());
   }

   public void get(QuadrupedTimedStep other)
   {
      other.setRobotQuadrant(getRobotQuadrant());
      other.setGoalPosition(getGoalPositionInternal());
      other.setGroundClearance(getGroundClearance());
      other.setTimeInterval(getTimeInterval());
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return getRobotQuadrant() == other.getRobotQuadrant() &&
            getGoalPositionInternal().epsilonEquals(other.getGoalPositionInternal(), epsilon) &&
            MathTools.epsilonEquals(getGroundClearance(), other.getGroundClearance(), epsilon) &&
            getTimeInterval().epsilonEquals(other.getTimeInterval(), epsilon);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      QuadrupedTimedStep other = (QuadrupedTimedStep) obj;

      if (getRobotQuadrant() != other.getRobotQuadrant())
         return false;
      if (getGroundClearance() != other.getGroundClearance())
         return false;
      if (!getGoalPosition().epsilonEquals(other.getGoalPosition(), 0.0))
         return false;
      if (!getTimeInterval().epsilonEquals(other.getTimeInterval(), 0.0))
         return false;

      return super.equals(other);
   }

   @Override
   public String toString()
   {
      String string = super.toString();
      string += "\nrobotQuadrant: " + getRobotQuadrant();
      string += "\ngoalPosition:" + getGoalPositionInternal();
      string += "\ngroundClearance: " + getGroundClearance();
      string += "\nstartTime: " + getTimeInterval().getStartTime();
      string += "\nendTime: " + getTimeInterval().getEndTime();
      return string;
   }
}

