package us.ihmc.commonWalkingControlModules.capturePoint.numerical;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ObjDoubleConsumer;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class CopTrajectory implements ObjDoubleConsumer<Point2DBasics>
{
   private final List<Point2DReadOnly> waypoints = new ArrayList<>();
   private final TDoubleList waypointTimes = new TDoubleArrayList();

   public CopTrajectory(SupportSeqence supportSeqence)
   {
      this(supportSeqence.getSupportPolygons(), supportSeqence.getSupportDurations());
   }

   public CopTrajectory(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportDurations)
   {
      double time = 0.0;

      // Initial waypoint at center of initial support.
      waypoints.add(new Point2D(supportPolygons.get(0).getCentroid()));
      waypointTimes.add(time);

      // Waypoint at end of support is as close to next support center as possible.
      for (int i = 0; i < supportPolygons.size() - 1; i++)
      {
         ConvexPolygon2DReadOnly polygon = supportPolygons.get(i);
         ConvexPolygon2DReadOnly nextPolygon = supportPolygons.get(i + 1);
         Point2DReadOnly nextCentroid = nextPolygon.getCentroid();
         polygon.orthogonalProjectionCopy(nextCentroid);
         time += supportDurations.get(i);
         if (polygon.isPointInside(nextCentroid))
         {
            waypoints.add(new Point2D(nextCentroid));
            waypointTimes.add(time);
         }
         else if (nextPolygon.isPointInside(polygon.getCentroid()))
         {
            waypoints.add(new Point2D(polygon.getCentroid()));
            waypointTimes.add(time);
         }
         else
         {
            waypoints.add(new Point2D(polygon.getCentroid()));
            waypointTimes.add(time);
            waypoints.add(new Point2D(nextCentroid));
            waypointTimes.add(time);
         }
      }

      // Last waypoint is at center of final support.
      time += supportDurations.get(supportDurations.size() - 1);
      waypoints.add(new Point2D(supportPolygons.get(supportPolygons.size() - 1).getCentroid()));
      waypointTimes.add(time);
   }

   @Override
   public void accept(Point2DBasics copToPack, double time)
   {
      if (time <= waypointTimes.get(0))
      {
         copToPack.set(waypoints.get(0));
         return;
      }

      if (time >= waypointTimes.get(waypoints.size() - 1))
      {
         copToPack.set(waypoints.get(waypoints.size() - 1));
         return;
      }

      int segmentIndex = 0;
      while (waypointTimes.get(segmentIndex + 1) < time)
         segmentIndex++;

      double timeInSegment = time - waypointTimes.get(segmentIndex);
      double segmentDuration = waypointTimes.get(segmentIndex + 1) - waypointTimes.get(segmentIndex);
      double percentInSegment = timeInSegment / segmentDuration;
      copToPack.interpolate(waypoints.get(segmentIndex), waypoints.get(segmentIndex + 1), percentInSegment);
   }
}
