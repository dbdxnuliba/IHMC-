package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.fusion.data.FusedSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.StereoREAParallelParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;

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

   /**
    * If this segment is big enough (isBigSegment = true), coplanar test is done with the closest label among this segment.
    */
   public static boolean areSuperPixelsCoplanar(FusedSuperPixelData fusedSuperPixel, boolean isBigPixel, SuperPixelData otherSuperPixel, double distanceThreshold)
   {
      Point3D closestSuperPixelCenter = new Point3D(fusedSuperPixel.getCenter());
      Vector3D closestSuperPixelNormal = new Vector3D(fusedSuperPixel.getNormal());
      if (isBigPixel)
      {
         double closestLabelDistance = Double.POSITIVE_INFINITY;
         int closestLabel = -1;
         for (int i = 0; i < fusedSuperPixel.getNumberOfComponentSuperPixels(); i++)
         {
            Point3DReadOnly labelCenter = fusedSuperPixel.getComponentSuperPixelCenter(i);
            double distanceToElementCenter = labelCenter.distance(otherSuperPixel.getCenter());
            if (distanceToElementCenter < closestLabelDistance)
            {
               closestLabelDistance = distanceToElementCenter;
               closestLabel = i;
            }
         }

         if (closestLabel > -1)
         {
            closestSuperPixelCenter.set(fusedSuperPixel.getComponentSuperPixelCenter(closestLabel));
            closestSuperPixelNormal.set(fusedSuperPixel.getComponentSuperPixelNormal(closestLabel));
         }
      }

      return SuperPixelTools.areSuperPixelsCoplanar(otherSuperPixel.getCenter(), otherSuperPixel.getNormal(), closestSuperPixelCenter,
                                                    closestSuperPixelNormal, distanceThreshold);
   }

   public static boolean areSuperPixelsCoplanar(Point3DReadOnly pixelACenter, Vector3DReadOnly pixelANormal, Point3DReadOnly pixelBCenter,
                                                Vector3DReadOnly pixelBNormal, double distanceThreshold)
   {
      double distanceToQueryFromHere = distancePlaneToPoint(pixelANormal, pixelACenter, pixelBCenter);
      double distanceToHereFromQuery = distancePlaneToPoint(pixelBNormal, pixelBCenter, pixelACenter);

      return (Math.abs(distanceToQueryFromHere) < distanceThreshold && Math.abs(distanceToHereFromQuery) < distanceThreshold);
   }

   public static void extendFusedSuperPixel(FusedSuperPixelData fusedSuperPixelToExtend, SuperPixelData superPixelToInclude,
                                           PlanarRegionPropagationParameters planarRegionPropagationParameters,
                                            SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      extendFusedSuperPixel(fusedSuperPixelToExtend, superPixelToInclude, planarRegionPropagationParameters.getExtendingDistanceThreshold(),
                            planarRegionPropagationParameters.getExtendingRadiusThreshold(), planarRegionPropagationParameters.isUpdateExtendedData(),
                            normalEstimationParameters);
   }

   public static void extendFusedSuperPixel(FusedSuperPixelData fusedSuperPixelToExtend, SuperPixelData superPixelToInclude, double maxDistanceFromPixel,
                                            double maxDistanceFromAnyPoint, boolean updateFusedSuperPixel,
                                            SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      for (Point3DReadOnly point : superPixelToInclude.getPointsInPixel())
      {
         double distance = SuperPixelTools.distancePlaneToPoint(fusedSuperPixelToExtend.getNormal(), fusedSuperPixelToExtend.getCenter(), point);
         if (distance < maxDistanceFromPixel)
         {
            if (fusedSuperPixelToExtend.getPointsInPixel().parallelStream().anyMatch(pointInSegment -> pointInSegment.distance(point) < maxDistanceFromAnyPoint))
            {
               // remove it from the other list
               superPixelToInclude.getPointsInPixel().remove(point);
               fusedSuperPixelToExtend.addPoint(point);
               break;
            }
         }
      }

      if (updateFusedSuperPixel)
      {
         fusedSuperPixelToExtend.updateNormal(normalEstimationParameters);
      }
   }

   public static double distancePlaneToPoint(Vector3DReadOnly planeNormal, Point3DReadOnly planeCenter, Point3DReadOnly point)
   {
      Vector3D centerVector = new Vector3D(planeCenter);
      double constantD = -planeNormal.dot(centerVector);

      if (planeNormal.lengthSquared() == 0)
         System.out.println("normalVector.lengthSquared() == 0");
      Vector3D pointVector = new Vector3D(point);
      return Math.abs(planeNormal.dot(pointVector) + constantD) / Math.sqrt(planeNormal.lengthSquared());
   }
}
