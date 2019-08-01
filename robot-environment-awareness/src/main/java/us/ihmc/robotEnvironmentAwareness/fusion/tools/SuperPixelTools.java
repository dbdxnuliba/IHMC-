package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SuperPixelData;

public class SuperPixelTools
{
   public static boolean areSuperPixelsParallel(SuperPixelData superPixelA, SuperPixelData superPixelB, double threshold)
   {
      return areSuperPixelsParallel(superPixelA.getNormal(), superPixelB.getNormal(), threshold);
   }

   public static boolean areSuperPixelsParallel(Vector3DReadOnly pixelANormal, Vector3DReadOnly pixelBNormal, double threshold)
   {
      return (Math.abs(pixelANormal.dot(pixelBNormal)) > threshold);
   }

   public static boolean areSuperPixelsCoplanar(Point3DReadOnly pixelACenter, Vector3DReadOnly pixelANormal, Point3DReadOnly pixelBCenter,
                                                Vector3DReadOnly pixelBNormal, double distanceThreshold)
   {
      double distanceToQueryFromHere = distancePlaneToPoint(pixelANormal, pixelACenter, pixelBCenter);
      double distanceToHereFromQuery = distancePlaneToPoint(pixelBNormal, pixelBCenter, pixelACenter);

      return (Math.abs(distanceToQueryFromHere) < distanceThreshold && Math.abs(distanceToHereFromQuery) < distanceThreshold);
   }

   private static double distancePlaneToPoint(Vector3DReadOnly planeNormal, Point3DReadOnly planeCenter, Point3DReadOnly point)
   {
      Vector3D centerVector = new Vector3D(planeCenter);
      double constantD = -planeNormal.dot(centerVector);

      if (planeNormal.lengthSquared() == 0)
         System.out.println("normalVector.lengthSquared() == 0");
      Vector3D pointVector = new Vector3D(point);
      return Math.abs(planeNormal.dot(pointVector) + constantD) / Math.sqrt(planeNormal.lengthSquared());
   }
}
