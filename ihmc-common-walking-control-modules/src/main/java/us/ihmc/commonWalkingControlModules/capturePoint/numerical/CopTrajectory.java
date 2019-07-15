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

   public CopTrajectory(SupportSeqence supportSeqence, Point2DReadOnly initialCop, double finalTransferDuration)
   {
      this(supportSeqence.getSupportPolygons(), supportSeqence.getSupportTimes(), initialCop, finalTransferDuration);
   }

   public CopTrajectory(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes, Point2DReadOnly initialCop,
                        double finalTransferDuration)
   {
      // Initial waypoint.
      if (initialCop == null)
         waypoints.add(new Point2D(supportPolygons.get(0).getCentroid()));
      else
         waypoints.add(new Point2D(initialCop));
      waypointTimes.add(0.0);

      // Waypoint at end of support is as close to next support center as possible.
      for (int i = 1; i < supportPolygons.size(); i++)
      {
         ConvexPolygon2DReadOnly previousPolygon = supportPolygons.get(i - 1);
         ConvexPolygon2DReadOnly polygon = supportPolygons.get(i);
         Point2DReadOnly lastWaypoint = waypoints.get(waypoints.size() - 1);
         Point2DReadOnly centroid = polygon.getCentroid();

         if (previousPolygon.isPointInside(centroid))
         {
            waypoints.add(new Point2D(centroid));
            waypointTimes.add(supportTimes.get(i));
         }
         else if (polygon.isPointInside(lastWaypoint))
         {
            waypoints.add(new Point2D(lastWaypoint));
            waypointTimes.add(supportTimes.get(i));
         }
         else
         {
            waypoints.add(new Point2D(lastWaypoint));
            waypointTimes.add(supportTimes.get(i));
            waypoints.add(new Point2D(centroid));
            waypointTimes.add(supportTimes.get(i));
         }
      }

      // Last waypoint is at center of final support.
      waypoints.add(new Point2D(supportPolygons.get(supportPolygons.size() - 1).getCentroid()));
      waypointTimes.add(supportTimes.get(supportTimes.size() - 1) + finalTransferDuration);
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
