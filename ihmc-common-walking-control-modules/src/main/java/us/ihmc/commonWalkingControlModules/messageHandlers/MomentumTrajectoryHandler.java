package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MomentumTrajectoryHandler extends EuclideanTrajectoryHandler
{
   public MomentumTrajectoryHandler(YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      super("AngularMomentum", yoTime, parentRegistry);
   }

   public void handleMomentumTrajectory(MomentumTrajectoryCommand command)
   {
      handleTrajectory(command.getAngularMomentumTrajectory());
   }

   /**
    * This method will pack the angular momentum trajectory for planning the ICP trajectory. The parameters {@code startTime} and {@code endTime} refer
    * to absolute controller time. To get the angular momentum trajectory from the current time to 1.0 seconds in the future the start time must
    * be the value of yoTime and the end time must be the value of yoTime + 1.0. The packed trajectory points will start and end with points which were
    * interpolated from the received waypoints. Any additional received points lying in the given interval are also included.
    * If the interval of interest is not available the trajectory to pack will be empty. The times of the packed trajectory points
    * will be relative to the start time of the interval.
    *
    * @param startTime is the controller time for the start of the interval for which the trajectory is packed
    * @param endTime is the controller time for the end of the interval for which the trajectory is packed
    * @param trajectoryToPack the trajectory will be packed in here
    */
   public void getAngularMomentumTrajectory(double startTime, double endTime, RecyclingArrayList<SimpleEuclideanTrajectoryPoint> trajectoryToPack)
   {
      trajectoryToPack.clear();
      if (!isWithinInterval(startTime) || !isWithinInterval(endTime))
      {
         return;
      }

      packDesiredsAtTime(startTime);
      SimpleEuclideanTrajectoryPoint interpolatedStartPoint = trajectoryToPack.add();
      interpolatedStartPoint.setTime(0.0);
      interpolatedStartPoint.getEuclideanWaypoint().setPosition(getPosition());
      interpolatedStartPoint.getEuclideanWaypoint().setLinearVelocity(getVelocity());

      int waypointIndex = 0;
      while(trajectoryPoints.get(waypointIndex).getTime() < startTime)
      {
         waypointIndex++;
      }

      while(trajectoryPoints.get(waypointIndex).getTime() < endTime)
      {
         SimpleEuclideanTrajectoryPoint waypoint = trajectoryToPack.add();
         waypoint.set(trajectoryPoints.get(waypointIndex));
         waypoint.subtractTimeOffset(startTime);
         waypointIndex++;
      }

      packDesiredsAtTime(endTime);
      SimpleEuclideanTrajectoryPoint interpolatedEndPoint = trajectoryToPack.add();
      interpolatedEndPoint.setTime(endTime - startTime);
      interpolatedEndPoint.getEuclideanWaypoint().setPosition(getPosition());
      interpolatedEndPoint.getEuclideanWaypoint().setLinearVelocity(getVelocity());
   }

   public boolean packDesiredAngularMomentumAtTime(double time, FrameVector3DBasics angularMomentumToPack, FrameVector3DBasics angularMomentumRateToPack)
   {
      if (!isWithinInterval(time))
      {
         angularMomentumToPack.setToNaN(ReferenceFrame.getWorldFrame());
         angularMomentumRateToPack.setToNaN(ReferenceFrame.getWorldFrame());
         return false;
      }

      packDesiredsAtTime(time);

      angularMomentumToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), getPosition());
      angularMomentumRateToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), getVelocity());
      return true;
   }
}
