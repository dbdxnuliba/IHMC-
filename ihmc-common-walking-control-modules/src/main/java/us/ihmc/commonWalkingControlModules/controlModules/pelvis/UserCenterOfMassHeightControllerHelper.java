package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class UserCenterOfMassHeightControllerHelper
{
   private final MultipleWaypointsPositionTrajectoryGenerator trajectoryGenerator;
   private final FrameEuclideanTrajectoryPoint lastPointAdded = new FrameEuclideanTrajectoryPoint();
   private final RecyclingArrayDeque<FrameEuclideanTrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyTaskspaceControlState.maxPoints,
                                                                                                           FrameEuclideanTrajectoryPoint.class,
                                                                                                           FrameEuclideanTrajectoryPoint::set);

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAcceleration = new FrameVector3D();

   private final ReferenceFrame worldFrame;
   private final ReferenceFrame comFrame;
   private final FramePoint3D comPosition = new FramePoint3D();

   private final String prefix;

   public UserCenterOfMassHeightControllerHelper(String warningPrefix, ReferenceFrame comFrame, YoVariableRegistry registry)
   {
      this.prefix = warningPrefix;
      this.comFrame = comFrame;
      worldFrame = ReferenceFrame.getWorldFrame();

      trajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("CenterOfMass",
                                                                             RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                             ReferenceFrame.getWorldFrame(),
                                                                             registry);
      trajectoryGenerator.clear(worldFrame);
   }

   private FramePoint3D getCurrentCoMPosition()
   {
      comPosition.setToZero(comFrame);
      comPosition.changeFrame(worldFrame);
      return comPosition;
   }

   public void holdCurrent()
   {
      clear();
      queueInitialPoint(getCurrentCoMPosition());
   }

   public void holdCurrentDesired()
   {
      getDesiredPosition(desiredPosition);
      clear();
      queueInitialPoint(desiredPosition);
   }

   public void goToPositionFromCurrent(FramePoint3DReadOnly position, double trajectoryTime)
   {
      holdCurrent();

      FrameEuclideanTrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredPosition.setIncludingFrame(position);
      desiredPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setPosition(desiredPosition);
   }

   public void goToPosition(FramePoint3DReadOnly position, double trajectoryTime)
   {
      holdCurrentDesired();

      FrameEuclideanTrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredPosition.setIncludingFrame(position);
      desiredPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setPosition(desiredPosition);
   }

   public void getDesiredPosition(FramePoint3D positionToPack)
   {
      if (trajectoryGenerator.isEmpty())
      {
         positionToPack.setIncludingFrame(getCurrentCoMPosition());
         positionToPack.changeFrame(trajectoryGenerator.getReferenceFrame());
      }
      else
      {
         trajectoryGenerator.getPosition(positionToPack);
      }
   }

   public boolean doAction(double timeInTrajectory)
   {
      boolean done = false;
      if (trajectoryGenerator.isDone() || trajectoryGenerator.getLastWaypointTime() <= timeInTrajectory)
      {
         done = fillAndReinitializeTrajectories();
      }

      trajectoryGenerator.compute(timeInTrajectory);
      trajectoryGenerator.getLinearData(desiredPosition, desiredVelocity, feedForwardAcceleration);

      desiredPosition.changeFrame(worldFrame);
      desiredVelocity.changeFrame(worldFrame);
      feedForwardAcceleration.changeFrame(worldFrame);

      return done;
   }

   private boolean fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         return true;
      }

      if (!trajectoryGenerator.isEmpty())
      {
         trajectoryGenerator.clear();
         lastPointAdded.changeFrame(trajectoryGenerator.getReferenceFrame());
         trajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      int currentNumberOfWaypoints = trajectoryGenerator.getCurrentNumberOfWaypoints();
      int pointsToAdd = RigidBodyTaskspaceControlState.maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         FrameEuclideanTrajectoryPoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd);
         trajectoryGenerator.appendWaypoint(pointToAdd);
      }

      trajectoryGenerator.initialize();
      return false;
   }

   public boolean handleTrajectoryCommand(EuclideanTrajectoryControllerCommand command)
   {
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE || isEmpty())
      {
         // Record the current desired position and the control frame pose.
         getDesiredPosition(desiredPosition);

         clear();

         trajectoryGenerator.changeFrame(command.getTrajectoryFrame());

         if (command.getTrajectoryPoint(0).getTime() > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
         {
            queueInitialPoint(desiredPosition);
         }
      }
      else if (command.getTrajectoryFrame() != trajectoryGenerator.getReferenceFrame())
      {
         LogTools.warn(prefix + "Was executing in " + trajectoryGenerator.getReferenceFrame() + " can not switch to " + command.getTrajectoryFrame()
               + " without override.");
         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryGenerator.getReferenceFrame());
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      return true;
   }

   public int getNumberOfPointsInQueue()
   {
      return pointQueue.size();
   }

   public int getNumberOfPointsInGenerator()
   {
      return trajectoryGenerator.getCurrentNumberOfWaypoints();
   }

   private void queueInitialPoint(FramePoint3D initialPosition)
   {
      initialPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      FrameEuclideanTrajectoryPoint initialPoint = pointQueue.addLast();
      initialPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      initialPoint.setTime(0.0);
      initialPoint.setPosition(initialPosition);
   }

   private boolean queuePoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      if (pointQueue.size() >= RigidBodyTaskspaceControlState.maxPoints)
      {
         LogTools.warn(prefix + "Reached maximum capacity of " + RigidBodyTaskspaceControlState.maxPoints + " can not execute trajectory.");
         return false;
      }

      pointQueue.addLast().setIncludingFrame(trajectoryPoint);
      return true;
   }

   private boolean checkTime(double time)
   {
      if (time <= getLastTrajectoryPointTime())
      {
         LogTools.warn(prefix + "Time in trajectory must be strictly increasing.");
         return false;
      }
      return true;
   }

   public boolean isEmpty()
   {
      if (!pointQueue.isEmpty())
      {
         return false;
      }
      return trajectoryGenerator.isDone();
   }

   public double getLastTrajectoryPointTime()
   {
      if (isEmpty())
      {
         return Double.NEGATIVE_INFINITY;
      }
      else if (pointQueue.isEmpty())
      {
         return trajectoryGenerator.getLastWaypointTime();
      }
      else
      {
         return pointQueue.peekLast().getTime();
      }
   }

   public void clear()
   {
      trajectoryGenerator.clear(worldFrame);
      pointQueue.clear();
   }

   public FramePoint3D getDesiredPosition()
   {
      return desiredPosition;
   }

   public FrameVector3D getDesiredVelocity()
   {
      return desiredVelocity;
   }

   public FrameVector3D getFeedForwardAcceleration()
   {
      return feedForwardAcceleration;
   }

}
