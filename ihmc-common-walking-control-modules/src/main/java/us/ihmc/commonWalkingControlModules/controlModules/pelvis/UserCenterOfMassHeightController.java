package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class UserCenterOfMassHeightController extends RigidBodyTaskspaceControlState
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

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   public UserCenterOfMassHeightController(ReferenceFrame comFrame, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, "CenterOfMass", yoTime, parentRegistry);

      prefix = "CenterOfMassPositionTaskspace";

      this.comFrame = comFrame;
      worldFrame = ReferenceFrame.getWorldFrame();

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      trajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("CenterOfMass",
                                                                             RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                             ReferenceFrame.getWorldFrame(),
                                                                             registry);
      trajectoryGenerator.clear(worldFrame);

      hideGraphics();
   }

   private FramePoint3D getCurrentCoMPosition()
   {
      comPosition.setToZero(comFrame);
      comPosition.changeFrame(worldFrame);
      return comPosition;
   }

   private void queueInitialPoint(FramePoint3D initialPosition)
   {
      initialPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      FrameEuclideanTrajectoryPoint initialPoint = pointQueue.addLast();
      initialPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      initialPoint.setTime(0.0);
      initialPoint.setPosition(initialPosition);
   }

   private void getDesiredPosition(FramePoint3D positionToPack)
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

   @Override
   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      queueInitialPoint(getCurrentCoMPosition());
   }

   @Override
   public void holdCurrentDesired()
   {
      getDesiredPosition(desiredPosition);
      clear();
      setTrajectoryStartTimeToCurrentTime();
      queueInitialPoint(desiredPosition);
   }

   @Override
   public void goToPoseFromCurrent(FramePose3DReadOnly pose, double trajectoryTime)
   {
      goToPositionFromCurrent(pose.getPosition(), trajectoryTime);
   }

   @Override
   public void goToPose(FramePose3DReadOnly pose, double trajectoryTime)
   {
      goToPosition(pose.getPosition(), trajectoryTime);
   }

   public void goToPositionFromCurrent(FramePoint3DReadOnly position, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
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
      clear();
      setTrajectoryStartTimeToCurrentTime();
      holdCurrentDesired();

      FrameEuclideanTrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredPosition.setIncludingFrame(position);
      desiredPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setPosition(desiredPosition);
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

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();

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

      numberOfPointsInQueue.set(pointQueue.size());
      numberOfPointsInGenerator.set(trajectoryGenerator.getCurrentNumberOfWaypoints());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      trajectoryDone.set(done);
      updateGraphics();
   }

   @Override
   public boolean handleTrajectoryCommand(EuclideanTrajectoryControllerCommand command)
   {
      // For a position controlled body the control frame orientation must remain identity so the current orientation can be used to
      // transform the desired from the old control frame position to the new.
      if (command.useCustomControlFrame() && !command.getControlFramePose().getRotation().isIdentity())
      {
         LogTools.warn("Specifying a control frame orientation for a body position controller is not supported!");
         clear();
         return false;
      }
      
      if (!handleCommandInternal(command))
      {
         clear();
         return false;
      }

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
         clear();
         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryGenerator.getReferenceFrame());
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
         {
            clear();
            return false;
         }
         if (!queuePoint(command.getTrajectoryPoint(i)))
         {
            clear();
            return false;
         }
      }

      return true;

   }

   @Override
   public void onExit()
   {
      hideGraphics();
      clear();
   }

   @Override
   public boolean isEmpty()
   {
      if (!pointQueue.isEmpty())
      {
         return false;
      }
      return trajectoryGenerator.isDone();
   }

   @Override
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

   private void clear()
   {
      trajectoryDone.set(true);
      resetLastCommandId();
      trajectoryGenerator.clear(worldFrame);
      pointQueue.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
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
