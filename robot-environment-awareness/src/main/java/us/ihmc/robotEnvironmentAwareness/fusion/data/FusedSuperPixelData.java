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
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class FusedSuperPixelData implements SuperPixelData
{
   private static final boolean addInParallel = true;

   private static final boolean USE_PCA_TO_UPDATE = true;
   private int id = PlanarRegion.NO_REGION_ID;
   private final TIntArrayList labels = new TIntArrayList();
   private final List<Point3DReadOnly> labelCenters = new ArrayList<>();
   private final List<Vector3DReadOnly> labelNormals = new ArrayList<>();

   private final Vector3D normal = new Vector3D();
   private final Point3D center = new Point3D();

   private final Vector3D standardDeviation = new Vector3D();
   private double normalVariance = Double.NaN;
   private int normalConsensus = Integer.MIN_VALUE;

   private double weight = 0.0;

   private final List<Point3DReadOnly> pointsInSegment = new ArrayList<>();
   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   public FusedSuperPixelData(RawSuperPixelData seedImageSegment)
   {
      id = seedImageSegment.getId();
      labels.add(seedImageSegment.getImageSegmentLabel());
      labelCenters.add(seedImageSegment.getCenter());
      labelNormals.add(seedImageSegment.getNormal());

      normal.set(seedImageSegment.getNormal());
      center.set(seedImageSegment.getCenter());

      pointsInSegment.addAll(seedImageSegment.getPointsInPixel());
   }

   @Override
   public Point3DReadOnly getCenter()
   {
      return center;
   }

   @Override
   public Vector3DReadOnly getNormal()
   {
      return normal;
   }

   @Override
   public void setCenter(Point3DReadOnly center)
   {
      this.center.set(center);
   }

   @Override
   public void setNormal(Vector3DReadOnly normal)
   {
      this.normal.set(normal);
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
      return pointsInSegment;
   }

   public void merge(RawSuperPixelData fusionDataSegment)
   {
      labels.add(fusionDataSegment.getImageSegmentLabel());
      labelCenters.add(fusionDataSegment.getCenter());
      labelNormals.add(fusionDataSegment.getNormal());

      pointsInSegment.addAll(fusionDataSegment.getPointsInPixel());

      if (USE_PCA_TO_UPDATE)
      {
         Stream<Point3DReadOnly> pointStream = addInParallel ? fusionDataSegment.getPointsInPixel().parallelStream() : fusionDataSegment.getPointsInPixel().stream();
         pointStream.forEach(pca::addDataPoint);
         pca.compute();

         pca.getMean(center);
         pca.getThirdVector(normal);

         if (normal.getZ() < 0.0)
            normal.negate();
      }
      else
      {
         double otherWeight = fusionDataSegment.getWeight();
         double totalWeight = weight + otherWeight;
         normal.setX((normal.getX() * weight + fusionDataSegment.getNormal().getX() * otherWeight) / totalWeight);
         normal.setY((normal.getY() * weight + fusionDataSegment.getNormal().getY() * otherWeight) / totalWeight);
         normal.setZ((normal.getZ() * weight + fusionDataSegment.getNormal().getZ() * otherWeight) / totalWeight);

         center.setX((center.getX() * weight + fusionDataSegment.getCenter().getX() * otherWeight) / totalWeight);
         center.setY((center.getY() * weight + fusionDataSegment.getCenter().getY() * otherWeight) / totalWeight);
         center.setZ((center.getZ() * weight + fusionDataSegment.getCenter().getZ() * otherWeight) / totalWeight);

         weight = totalWeight;
      }
   }

   public void extend(RawSuperPixelData fusionDataSegment, double threshold, boolean updateNodeData, double extendingThreshold,
                      SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      for (Point3DReadOnly point : fusionDataSegment.getPointsInPixel())
      {
         double distance = distancePlaneToPoint(normal, center, point);
         if (distance < threshold)
         {
            for (Point3DReadOnly pointInSegment : pointsInSegment)
            {
               if (pointInSegment.distance(point) < extendingThreshold)
               {
                  pointsInSegment.add(point);
                  break;
               }
            }
         }
      }

      if (updateNodeData)
      {
//         if (normalEstimationParameters.updateUsingPCA())
            SuperPixelNormalEstimationTools.updateUsingPCA(this, pointsInSegment, StereoREAParallelParameters.addPointsToPCAWhenExtendingInParallel);
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
      return labels;
   }


   /**
    * If this segment is big enough (isBigSegment = true), coplanar test is done with the closest label among this segment.
    */
   public boolean isCoplanar(RawSuperPixelData fusionDataSegment, double threshold, boolean isBigSegment)
   {
      Point3D nodeDataCenter = new Point3D(center);
      Vector3D nodeDataNormal = new Vector3D(normal);
      if (isBigSegment)
      {
         double min = Double.POSITIVE_INFINITY;
         double cur = 0;
         int closestLabel = -1;
         for (int i = 0; i < labelCenters.size(); i++)
         {
            Point3DReadOnly labelCenter = labelCenters.get(i);
            cur = labelCenter.distance(fusionDataSegment.getCenter());
            if (cur < min)
            {
               min = cur;
               closestLabel = i;
            }
         }
         if (closestLabel > 0)
         {
            nodeDataCenter.set(labelCenters.get(closestLabel));
            nodeDataNormal.set(labelNormals.get(closestLabel));
         }
      }

      double distanceFromSegment = distancePlaneToPoint(fusionDataSegment.getNormal(), fusionDataSegment.getCenter(), nodeDataCenter);
      double distanceToSegment = distancePlaneToPoint(nodeDataNormal, nodeDataCenter, fusionDataSegment.getCenter());

      if (Math.abs(distanceFromSegment) < threshold && Math.abs(distanceToSegment) < threshold)
         return true;
      else
         return false;
   }

   public boolean isParallel(RawSuperPixelData fusionDataSegment, double threshold)
   {
      if (Math.abs(fusionDataSegment.getNormal().dot(getNormal())) > threshold)
         return true;
      else
         return false;
   }

   private static double distancePlaneToPoint(Vector3DReadOnly normalVector, Point3DReadOnly center, Point3DReadOnly point)
   {
      Vector3D centerVector = new Vector3D(center);
      double constantD = -normalVector.dot(centerVector);

      if (normalVector.lengthSquared() == 0)
         System.out.println("normalVector.lengthSquared() == 0");
      Vector3D pointVector = new Vector3D(point);
      return Math.abs(normalVector.dot(pointVector) + constantD) / Math.sqrt(normalVector.lengthSquared());
   }
}
