package us.ihmc.robotEnvironmentAwareness.fusion.data;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.List;

public interface SuperPixelData
{
   Point3DReadOnly getCenter();

   Vector3DReadOnly getNormal();

   List<Point3DReadOnly> getPointsInPixel();

   void setCenter(Point3DReadOnly center);

   void setNormal(Vector3DReadOnly normal);

   void setStandardDeviation(Vector3DReadOnly standardDeviation);

   void setNormalQuality(double variance, int consensus);

   default boolean isNormalSet()
   {
      return !getNormal().containsNaN();
   }

   default boolean isCenterSet()
   {
      return !getCenter().containsNaN();
   }
}
