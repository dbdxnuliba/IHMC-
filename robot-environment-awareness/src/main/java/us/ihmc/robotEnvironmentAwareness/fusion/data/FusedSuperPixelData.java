package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.StereoREAParallelParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SuperPixelNormalEstimationTools;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SuperPixelTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class FusedSuperPixelData implements SuperPixelData
{
   private static final boolean addInParallel = true;
   private static final boolean USE_PCA_TO_UPDATE = true;

   private final int id;
   private final TIntArrayList componentPixelLabels = new TIntArrayList();
   private final List<Point3DReadOnly> componentPixelCenters = new ArrayList<>();
   private final List<Vector3DReadOnly> componentPixelNormals = new ArrayList<>();

   private final Vector3D fusedNormal = new Vector3D();
   private final Point3D fusedCenter = new Point3D();

   private final Vector3D standardDeviation = new Vector3D();
   private double normalVariance = Double.NaN;
   private int normalConsensus = Integer.MIN_VALUE;

   private double weight = 0.0;

   private final List<Point3DReadOnly> allPointsInPixel = new ArrayList<>();
   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   public FusedSuperPixelData(RawSuperPixelData seedImageSegment)
   {
      id = seedImageSegment.getId();
      componentPixelLabels.add(seedImageSegment.getImageSegmentLabel());
      componentPixelCenters.add(seedImageSegment.getCenter());
      componentPixelNormals.add(seedImageSegment.getNormal());

      fusedNormal.set(seedImageSegment.getNormal());
      fusedCenter.set(seedImageSegment.getCenter());

      allPointsInPixel.addAll(seedImageSegment.getPointsInPixel());
   }

   @Override
   public Point3DReadOnly getCenter()
   {
      return fusedCenter;
   }

   @Override
   public Vector3DReadOnly getNormal()
   {
      return fusedNormal;
   }

   @Override
   public void setCenter(Point3DReadOnly center)
   {
      this.fusedCenter.set(center);
   }

   @Override
   public void setNormal(Vector3DReadOnly normal)
   {
      this.fusedNormal.set(normal);
   }

   @Override
   public void setStandardDeviation(Vector3DReadOnly standardDeviation)
   {
      this.standardDeviation.set(standardDeviation);
   }

   @Override
   public void setNormalQuality(double normalVariance, int normalConsensus)
   {
      this.normalVariance = normalVariance;
      this.normalConsensus = normalConsensus;
   }

   @Override
   public List<Point3DReadOnly> getPointsInPixel()
   {
      return allPointsInPixel;
   }

   public void merge(RawSuperPixelData fusionDataSegment)
   {
      componentPixelLabels.add(fusionDataSegment.getImageSegmentLabel());
      componentPixelCenters.add(fusionDataSegment.getCenter());
      componentPixelNormals.add(fusionDataSegment.getNormal());

      allPointsInPixel.addAll(fusionDataSegment.getPointsInPixel());

      updateFusedParameters(fusionDataSegment);
   }

   private void updateFusedParameters(RawSuperPixelData fusionDataSegment)
   {
      if (USE_PCA_TO_UPDATE)
      {
         Stream<Point3DReadOnly> pointStream = addInParallel ? fusionDataSegment.getPointsInPixel().parallelStream() : fusionDataSegment.getPointsInPixel().stream();
         pointStream.forEach(pca::addDataPoint);
         pca.compute();

         pca.getMean(fusedCenter);
         pca.getThirdVector(fusedNormal);

         if (fusedNormal.getZ() < 0.0)
            fusedNormal.negate();
      }
      else
      {
         double otherWeight = fusionDataSegment.getWeight();
         double totalWeight = weight + otherWeight;

         // TODO do this via interpolation
         fusedNormal.setX((fusedNormal.getX() * weight + fusionDataSegment.getNormal().getX() * otherWeight) / totalWeight);
         fusedNormal.setY((fusedNormal.getY() * weight + fusionDataSegment.getNormal().getY() * otherWeight) / totalWeight);
         fusedNormal.setZ((fusedNormal.getZ() * weight + fusionDataSegment.getNormal().getZ() * otherWeight) / totalWeight);

         fusedCenter.setX((fusedCenter.getX() * weight + fusionDataSegment.getCenter().getX() * otherWeight) / totalWeight);
         fusedCenter.setY((fusedCenter.getY() * weight + fusionDataSegment.getCenter().getY() * otherWeight) / totalWeight);
         fusedCenter.setZ((fusedCenter.getZ() * weight + fusionDataSegment.getCenter().getZ() * otherWeight) / totalWeight);

         weight = totalWeight;
      }
   }

   public void extend(RawSuperPixelData fusionDataSegment, double threshold, boolean updateNodeData, double extendingThreshold,
                      SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      for (Point3DReadOnly point : fusionDataSegment.getPointsInPixel())
      {
         double distance = distancePlaneToPoint(fusedNormal, fusedCenter, point);
         if (distance < threshold)
         {
            for (Point3DReadOnly pointInSegment : allPointsInPixel)
            {
               if (pointInSegment.distance(point) < extendingThreshold)
               {
                  allPointsInPixel.add(point);
                  break;
               }
            }
         }
      }

      if (updateNodeData)
      {
//         if (normalEstimationParameters.updateUsingPCA())
            SuperPixelNormalEstimationTools.updateUsingPCA(this, allPointsInPixel, StereoREAParallelParameters.addPointsToPCAWhenExtendingInParallel);
//         else
//            SuperPixelNormalEstimationTools.updateUsingRansac(this, pointsInSegment, normalEstimationParameters);
      }
   }

   public int getId()
   {
      return id;
   }

   public TIntArrayList getLabels()
   {
      return componentPixelLabels;
   }


   /**
    * If this segment is big enough (isBigSegment = true), coplanar test is done with the closest label among this segment.
    */
   public boolean isCoplanar(RawSuperPixelData rawSuperPixelData, double threshold, boolean isBigSegment)
   {
      Point3D closestSuperPixelCenter = new Point3D(fusedCenter);
      Vector3D closestSuperPixelNormal = new Vector3D(fusedNormal);
      if (isBigSegment)
      {
         double closestLabelDistance = Double.POSITIVE_INFINITY;
         int closestLabel = -1;
         for (int i = 0; i < componentPixelCenters.size(); i++)
         {
            Point3DReadOnly labelCenter = componentPixelCenters.get(i);
            double distanceToElementCenter = labelCenter.distance(rawSuperPixelData.getCenter());
            if (distanceToElementCenter < closestLabelDistance)
            {
               closestLabelDistance = distanceToElementCenter;
               closestLabel = i;
            }
         }

         if (closestLabel > -1)
         {
            closestSuperPixelCenter.set(componentPixelCenters.get(closestLabel));
            closestSuperPixelNormal.set(componentPixelNormals.get(closestLabel));
         }
      }

      return SuperPixelTools.areSuperPixelsCoplanar(rawSuperPixelData.getCenter(), rawSuperPixelData.getNormal(), closestSuperPixelCenter,
                                                    closestSuperPixelNormal, threshold);
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
