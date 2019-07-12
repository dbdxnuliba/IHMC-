package us.ihmc.commonWalkingControlModules.capturePoint.numerical;

import java.util.List;
import java.util.function.ObjDoubleConsumer;

import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class SimpleTrajectory implements ObjDoubleConsumer<Point2DBasics>
{
   private final List<? extends Point2DReadOnly> waypoints;
   private final double duration;
   private final double intervalTime;

   public SimpleTrajectory(List<? extends Point2DReadOnly> waypoints, double duration)
   {
      this.waypoints = waypoints;
      this.duration = duration;
      this.intervalTime = duration / (waypoints.size() - 1);
   }

   @Override
   public void accept(Point2DBasics pointToPack, double time)
   {
      if (time >= duration)
      {
         pointToPack.set(waypoints.get(waypoints.size() - 1));
         return;
      }
      if (time <= 0)
      {
         pointToPack.set(waypoints.get(0));
         return;
      }

      int segmentIndex = 0;
      while (intervalTime * (1 + segmentIndex) < time)
         segmentIndex++;

      double timeInSegment = time - intervalTime * segmentIndex;
      double percentInSegment = timeInSegment / intervalTime;
      pointToPack.interpolate(waypoints.get(segmentIndex), waypoints.get(segmentIndex + 1), percentInSegment);
   }
}
