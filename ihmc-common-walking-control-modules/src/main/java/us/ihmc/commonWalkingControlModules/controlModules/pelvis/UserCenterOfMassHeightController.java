package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class UserCenterOfMassHeightController extends RigidBodyTaskspaceControlState
{

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final UserCenterOfMassHeightControllerHelper positionHelper;

   public UserCenterOfMassHeightController(ReferenceFrame comFrame, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, "CenterOfMass", yoTime, parentRegistry);

      String prefix = "CenterOfMassPositionTaskspace";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      positionHelper = new UserCenterOfMassHeightControllerHelper(prefix, comFrame, registry);

      hideGraphics();
   }

   @Override
   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.holdCurrent();
   }

   @Override
   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.holdCurrentDesired();
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
      positionHelper.goToPositionFromCurrent(position, trajectoryTime);
   }

   public void goToPosition(FramePoint3DReadOnly position, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.goToPosition(position, trajectoryTime);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();
      trajectoryDone.set(positionHelper.doAction(timeInTrajectory));

      numberOfPointsInQueue.set(positionHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(positionHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

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
         positionHelper.clear();
         return false;
      }

      if (handleCommandInternal(command) && positionHelper.handleTrajectoryCommand(command))
      {
         return true;
      }

      clear();
      positionHelper.clear();
      return false;
   }

   @Override
   public void onExit()
   {
      positionHelper.clear();
      hideGraphics();
      clear();
   }

   @Override
   public boolean isEmpty()
   {
      return positionHelper.isEmpty();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return positionHelper.getLastTrajectoryPointTime();
   }

   private void clear()
   {
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      trajectoryDone.set(true);
      resetLastCommandId();
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
   }

   public FramePoint3D getDesiredPosition()
   {
      return positionHelper.getDesiredPosition();
   }

   public FrameVector3D getDesiredVelocity()
   {
      return positionHelper.getDesiredVelocity();
   }

   public FrameVector3D getFeedForwardAcceleration()
   {
      return positionHelper.getFeedForwardAcceleration();
   }
}
