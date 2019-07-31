package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelImage;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.StereoREAParallelParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public class SegmentationRawDataFiltering
{
   public static void filterOutFlyingPoints(RawSuperPixelData rawSuperPixel, SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters)
   {
      if (!segmentationRawDataFilteringParameters.isEnableFilterFlyingPoint())
         return;

      double flyingPointThreshold = segmentationRawDataFilteringParameters.getFlyingPointThreshold();
      int minimumNeighborOfFlyingPointNeighbors = segmentationRawDataFilteringParameters.getMinimumNumberOfFlyingPointNeighbors();

      List<Point3DReadOnly> points = rawSuperPixel.getPoints();
      List<Point3DReadOnly> filteredPoints = new ArrayList<>();
      for (Point3DReadOnly point : points)
      {
         if (isPointCloseEnoughToOtherPoints(flyingPointThreshold, minimumNeighborOfFlyingPointNeighbors, point, points))
            filteredPoints.add(point);
      }

      points.clear();
      points.addAll(filteredPoints);
   }

   private static boolean isPointCloseEnoughToOtherPoints(double minDistanceToNeighbor, int requiredNumberOfNeighbors, Point3DReadOnly point,
                                                          List<Point3DReadOnly> otherPoints)
   {
      requiredNumberOfNeighbors = Math.max(requiredNumberOfNeighbors, 0);
      Stream<Point3DReadOnly> pointStream = StereoREAParallelParameters.useParallelStreamsForFiltering ? otherPoints.parallelStream() : otherPoints.stream();
      // subtract 1 to remove self
      int numberOfNeighbors = ((int) pointStream.filter(otherPoint -> point.distance(otherPoint) < minDistanceToNeighbor).count()) - 1;
      return numberOfNeighbors > requiredNumberOfNeighbors;
   }

   /**
    * Scaled threshold is used according to the v value of the segment center.
    */
   public static void updateSparsity(RawSuperPixelImage superPixelImage, SegmentationRawDataFilteringParameters rawDataFilteringParameters)
   {
      List<RawSuperPixelData> superPixelData = superPixelImage.getSuperPixelData();
      Stream<RawSuperPixelData> superPixelStream = StereoREAParallelParameters.updateSparsityInParallel ? superPixelData.parallelStream() : superPixelData.stream();
      superPixelStream.forEach(superPixel -> updateSparsity(superPixel, rawDataFilteringParameters, superPixelImage.getImageHeight()));
   }

   private static void updateSparsity(RawSuperPixelData rawSuperPixel, SegmentationRawDataFilteringParameters rawDataFilteringParameters, int imageHeight)
   {
      double sparseLowerThreshold = rawDataFilteringParameters.getMinimumSparseThreshold();
      double sparseUpperThreshold = sparseLowerThreshold * rawDataFilteringParameters.getMaximumSparsePropotionalRatio();

      double alpha = 1 - rawSuperPixel.getSegmentCenter().getY() / imageHeight;
      double threshold = alpha * (sparseUpperThreshold - sparseLowerThreshold) + sparseLowerThreshold;
      updateSparsity(rawSuperPixel, threshold);

      if (rawDataFilteringParameters.isEnableFilterCentrality())
         SegmentationRawDataFiltering.updateSparsityFromCentrality(rawSuperPixel, rawDataFilteringParameters);
      if (rawDataFilteringParameters.isEnableFilterEllipticity())
         SegmentationRawDataFiltering.updateSparsityFromEllipticity(rawSuperPixel, rawDataFilteringParameters);
   }

   private static void updateSparsity(RawSuperPixelData rawSuperPixelData, double threshold)
   {
      rawSuperPixelData.setIsSparse(rawSuperPixelData.getStandardDeviation().getZ() > threshold);
   }

   /**
    * Not to be sparse,
    * The number of points inside the area within a radius from the center should be over the ratio to all points in segment.
    */
   public static void updateSparsityFromCentrality(RawSuperPixelData rawSuperPixelData, SegmentationRawDataFilteringParameters rawDataFilteringParameters)
   {
      updateSparsityFromCentrality(rawSuperPixelData, rawDataFilteringParameters.getCentralityRadius(), rawDataFilteringParameters.getCentralityThreshold());
   }

   /**
    * Not to be sparse,
    * The number of points inside the area within a radius from the center should be over the ratio to all points in segment.
    */
   public static void updateSparsityFromCentrality(RawSuperPixelData rawSuperPixelData, double radius, double threshold)
   {
      if (!rawSuperPixelData.isSparse())
      {
         Stream<Point3DReadOnly> pointStream = StereoREAParallelParameters.useParallelStreamsForFiltering ? rawSuperPixelData.getPoints().parallelStream() : rawSuperPixelData.getPoints().stream();
         if (pointStream.filter(point -> rawSuperPixelData.getCenter().distance(point) < radius).count() < threshold * rawSuperPixelData.getWeight())
            rawSuperPixelData.setIsSparse(true);
//         int numberOfInliers = 0;
//         for (Point3DReadOnly point : rawSuperPixelData.getPoints())
//         {
//            double distance = rawSuperPixelData.getCenter().distance(point);
//            if (distance < radius)
//               numberOfInliers++;
//         }
//
//         if (numberOfInliers < threshold * rawSuperPixelData.getWeight())
//         {
//            rawSuperPixelData.setIsSparse(true);
//         }
      }
   }

   /**
    * Not to be sparse,
    * Secondary axis should be over the minLength.
    * Secondary axis should be within threshold * primary axis length.
    */
   public static void updateSparsityFromEllipticity(RawSuperPixelData rawSuperPixelData, SegmentationRawDataFilteringParameters rawDataFilteringParameters)
   {
      updateSparsityFromEllipticity(rawSuperPixelData, rawDataFilteringParameters.getEllipticityMinimumLength(), rawDataFilteringParameters.getEllipticityThreshold());
   }

   /**
    * Not to be sparse,
    * Secondary axis should be over the minLength.
    * Secondary axis should be within threshold * primary axis length.
    */
   public static void updateSparsityFromEllipticity(RawSuperPixelData rawSuperPixel, double minLength, double threshold)
   {
      if (!rawSuperPixel.isSparse())
      {
         double lengthPrimary = rawSuperPixel.getStandardDeviation().getX();
         double lengthSecondary = rawSuperPixel.getStandardDeviation().getY();
         if (lengthSecondary < minLength || lengthSecondary > lengthPrimary * threshold)
         {
            rawSuperPixel.setIsSparse(true);
         }
      }
   }
}
