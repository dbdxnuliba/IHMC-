package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public class CollinearForceMotionPlannerSegment
{
   public double duration;
   public final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();

   public void setSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      this.supportPolygon.set(supportPolygonToSet);
   }

   public void getSupportPolygon(ConvexPolygon2D convexPolygonToSet)
   {
      convexPolygonToSet.set(supportPolygon);
   }

   public void setSegmentDuration(double segmentDuration)
   {
      this.duration = segmentDuration;
   }

   public boolean isSupported()
   {
      return supportPolygon.getNumberOfVertices() > 0;
   }

   public void getSupportPolygonCentroid(Point2DBasics centroidToPack)
   {
      supportPolygon.getCentroid(centroidToPack);
   }

   public void getSupportPolygonCentroid(Point3DBasics centroidToPack)
   {
      centroidToPack.set(supportPolygon.getCentroid(), 0.0);
   }
   
   public double getSegmentDuration()
   {
      return duration;
   }

   public String toString()
   {
      return "Duration: " + duration + ", SupportPolygon: " + supportPolygon.toString();
   }
}