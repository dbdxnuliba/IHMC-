package us.ihmc.commonWalkingControlModules.capturePoint.numerical;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.function.ObjDoubleConsumer;

import org.apache.commons.math3.util.Precision;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class CopTrajectory implements ObjDoubleConsumer<Point2DBasics>
{
   /**
    * Distance for checks whether CoP waypoint is within a support. This should be small but positive to make sure that
    * a toe off polygon for example is considered connected to the full foot polygon causing the CoP to move there
    * instead of jump.
    */
   private static final double EPSILON = 0.001;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final RecyclingArrayList<Point2D> waypoints = new RecyclingArrayList<>(50, Point2D.class);
   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();

   private final List<YoFramePoint2D> yoWaypoints = new ArrayList<>();

   public CopTrajectory()
   {
      this(null, null);
   }

   public CopTrajectory(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsRegistry)
   {
      for (int i = 0; i < 10; i++)
         yoWaypoints.add(new YoFramePoint2D("CopWaypoint" + i, ReferenceFrame.getWorldFrame(), registry));

      if (graphicsRegistry != null)
      {
         for (int i = 0; i < yoWaypoints.size(); i++)
         {
            YoArtifactPosition artifact = new YoArtifactPosition("CopWaypoint" + i, yoWaypoints.get(i), GraphicType.DIAMOND, Color.CYAN, 0.002);
            graphicsRegistry.registerArtifact("CopWaypoints", artifact);
         }
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void set(Point2DReadOnly constantCop)
   {
      clear();

      waypoints.add().set(constantCop);
      waypointTimes.add(0.0);

      updateViz();
   }

   public void set(List<ConvexPolygon2D> supportPolygons, TDoubleList supportTimes, double finalTransferDuration)
   {
      update(supportPolygons, supportTimes, finalTransferDuration, 0.0, supportPolygons.get(0).getCentroid());
   }

   public void update(SupportSeqence supportSeqence, double finalTransferDuration, double timeInSequence, Point2DReadOnly initialCop)
   {
      update(supportSeqence.getSupportPolygons(), supportSeqence.getSupportTimes(), finalTransferDuration, timeInSequence, initialCop);
   }

   public void update(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes, double finalTransferDuration, double timeInSequence,
                      Point2DReadOnly initialCop)
   {
      clear();

      Point2DReadOnly centroid = supportPolygons.get(0).getCentroid();
      waypoints.add().set(centroid);
      waypointTimes.add(0.0);

      for (int i = 1; i < supportPolygons.size(); i++)
         addPolygon(supportPolygons, supportTimes, finalTransferDuration, i);

      // Last waypoint is at center of final support.
      waypoints.add().set(supportPolygons.get(supportPolygons.size() - 1).getCentroid());
      waypointTimes.add(supportTimes.get(supportTimes.size() - 1) + finalTransferDuration);

      addInitialCop(timeInSequence, initialCop);

      updateViz();
   }

   private void addInitialCop(double timeInSequence, Point2DReadOnly initialCop)
   {
      int index = 1;
      while (index < waypointTimes.size() && waypointTimes.get(index) < timeInSequence)
         index++;

      if (Precision.equals(waypointTimes.get(index - 1), timeInSequence))
      {
         waypoints.get(index - 1).set(initialCop);
      }
      else
      {
         // Index points to the last waypoint that is before timeInSequence
         waypoints.insertAtIndex(index).set(initialCop);
         waypointTimes.insert(index, timeInSequence);
      }
   }

   private void addPolygon(List<? extends ConvexPolygon2DReadOnly> supportPolygons, TDoubleList supportTimes, double finalTransferDuration, int i)
   {
      ConvexPolygon2DReadOnly previousPolygon = supportPolygons.get(i - 1);
      ConvexPolygon2DReadOnly polygon = supportPolygons.get(i);
      Point2DReadOnly centroid = polygon.getCentroid();
      Point2DReadOnly lastWaypoint = waypoints.get(waypoints.size() - 1);

      // Add a waypoint at the start of this polygon. This waypoint must be both in this polygon and the previous one.
      if (previousPolygon.isPointInside(centroid, EPSILON))
      {
         waypoints.add().set(centroid);
         waypointTimes.add(supportTimes.get(i));
      }
      else if (polygon.isPointInside(lastWaypoint, EPSILON))
      {
         waypoints.add().set(lastWaypoint);
         waypointTimes.add(supportTimes.get(i));
      }
      else
      {
         LogTools.warn("Discontinuous support sequence! Expect invalid plans.");
         waypoints.add().set(centroid);
         waypointTimes.add(supportTimes.get(i));
      }

      // Add the polygons centroid as a waypoint halfway through this support polygons duration.
      if (i < supportTimes.size() - 1)
      {
         double supportDuration = supportTimes.get(i + 1) - supportTimes.get(i);
         waypoints.add().set(centroid);
         waypointTimes.add(supportTimes.get(i) + supportDuration / 2.0);
      }
      else
      {
         waypoints.add().set(centroid);
         waypointTimes.add(supportTimes.get(i) + finalTransferDuration / 2.0);
      }
   }

   private void updateViz()
   {
      int max = Math.min(yoWaypoints.size(), waypoints.size());
      for (int i = 0; i < max; i++)
         yoWaypoints.get(i).set(waypoints.get(i));
      for (int i = max; i < yoWaypoints.size(); i++)
         yoWaypoints.get(i).setToNaN();
   }

   private void clear()
   {
      waypoints.clear();
      waypointTimes.reset();
   }

   @Override
   public void accept(Point2DBasics copToPack, double time)
   {
      if (time < waypointTimes.get(0))
         throw new RuntimeException("Requested at time " + time + " but first waypoint was at " + waypointTimes.get(0));

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
