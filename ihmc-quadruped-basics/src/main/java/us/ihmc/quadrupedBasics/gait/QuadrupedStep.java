package us.ihmc.quadrupedBasics.gait;

import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedStepCommand;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class QuadrupedStep
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_RIGHT;
   private Point3D goalPosition = new Point3D(0.0, 0.0, 0.0);
   private double groundClearance = 0.0;
   private TrajectoryType trajectoryType = null;
   private final RecyclingArrayList<Point3D> customPositionWaypoints = new RecyclingArrayList<Point3D>(2, Point3D.class);

   public QuadrupedStep()
   {
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, FramePoint3D goalPosition, double groundClearance)
   {
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, Point3DBasics goalPosition, double groundClearance)
   {
      this();
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
   }

   public QuadrupedStep(QuadrupedStep other)
   {
      this(other.getRobotQuadrant(), other.getGoalPositionInternal(), other.getGroundClearance());
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
   
   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
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

   public void set(QuadrupedStep other)
   {
      setRobotQuadrant(other.getRobotQuadrant());
      setGoalPosition(other.getGoalPositionInternal());
      setGroundClearance(other.getGroundClearance());
      setTrajectoryType(other.getTrajectoryType());
      setCustomPositionWaypoints(other.getCustomPositionWaypoints());
   }

   public void set(QuadrupedStepCommand command)
   {
      setRobotQuadrant(command.getRobotQuadrant());
      setGoalPosition(command.getGoalPosition());
      setGroundClearance(command.getGroundClearance());
      setTrajectoryType(command.getTrajectoryType());
      setCustomPositionWaypoints(command.getCustomPositionWaypoints());
   }

   public void get(QuadrupedTimedStep other)
   {
      other.setRobotQuadrant(getRobotQuadrant());
      other.setGoalPosition(getGoalPositionInternal());
      other.setGroundClearance(getGroundClearance());
      other.setTrajectoryType(getTrajectoryType());
      other.setCustomPositionWaypoints(getCustomPositionWaypoints());
   }
   
   public void setCustomPositionWaypoints(List<? extends Point3DBasics> customPositionWaypoints)
   {
      this.customPositionWaypoints.clear();
      for(int i = 0; i < customPositionWaypoints.size(); i++)
      {
         this.customPositionWaypoints.add().set(customPositionWaypoints.get(i));
      }
   }
   
   public List<? extends Point3DBasics> getCustomPositionWaypoints()
   {
      return customPositionWaypoints;
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      boolean areQuadrantsEqual = getRobotQuadrant() == other.getRobotQuadrant();
      boolean areGoalPositionsEqual = getGoalPositionInternal().epsilonEquals(other.getGoalPositionInternal(), epsilon);
      boolean areGroundClearancesEqual = MathTools.epsilonEquals(getGroundClearance(), other.getGroundClearance(), epsilon);
      boolean areTrajectoryTypesEqual = getTrajectoryType() == other.getTrajectoryType();
      boolean areCustomWaypointsEqual = getCustomPositionWaypoints().size() == other.getCustomPositionWaypoints().size();
      for(int i = 0; i < Math.min(getCustomPositionWaypoints().size(), other.getCustomPositionWaypoints().size()); i++)
      {
         areCustomWaypointsEqual = areCustomWaypointsEqual && getCustomPositionWaypoints().get(i).epsilonEquals(other.getCustomPositionWaypoints().get(i), epsilon);
      }
      
      return areQuadrantsEqual &&
             areGoalPositionsEqual &&
             areGroundClearancesEqual &&
             areTrajectoryTypesEqual && 
             areCustomWaypointsEqual;
   }

   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      QuadrupedStep other = (QuadrupedStep) obj;

      if (getRobotQuadrant() != other.getRobotQuadrant())
         return false;
      if (getGroundClearance() != other.getGroundClearance())
         return false;
      
      if(getCustomPositionWaypoints().size() != other.getCustomPositionWaypoints().size())
         return false;
      
      for(int i = 0; i < getCustomPositionWaypoints().size(); i++)
      {
         if(!getCustomPositionWaypoints().get(i).equals(other.getCustomPositionWaypoints().get(i)))
               return false;
      }
      
      if (trajectoryType != other.trajectoryType)
         return false;

      return getGoalPosition().geometricallyEquals(other.getGoalPosition(), 0.0);
   }

   @Override public String toString()
   {
      String string = super.toString();
      string += "\nrobotQuadrant: " + getRobotQuadrant();
      string += "\ngoalPosition:" + getGoalPositionInternal();
      string += "\ngroundClearance: " + getGroundClearance();
      string += "\ntrajectoryType: " + getTrajectoryType();
      string += "\ncustomWaypoint: " + getCustomPositionWaypoints();
      return string;
   }
}
