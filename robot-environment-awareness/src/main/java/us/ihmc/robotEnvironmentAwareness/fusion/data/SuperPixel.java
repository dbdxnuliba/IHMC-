package us.ihmc.robotEnvironmentAwareness.fusion.data;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface SuperPixel
{
   Point3DReadOnly getCenter();

   Vector3DReadOnly getNormal();

   void setCenter(Point3DReadOnly center);

   void setNormal(Vector3DReadOnly normal);

   void setStandardDeviation(Vector3DReadOnly standardDeviation);
}
