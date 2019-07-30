package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class SegmentationNodeData
{
   private static final boolean USE_PCA_TO_UPDATE = true;
   private int id = PlanarRegion.NO_REGION_ID;
   private final TIntArrayList labels = new TIntArrayList();
   private final List<Point3DReadOnly> labelCenters = new ArrayList<>();
   private final List<Vector3DReadOnly> labelNormals = new ArrayList<>();

   private final Vector3D normal = new Vector3D();
   private final Point3D center = new Point3D();

   private double weight = 0.0;

   private final List<Point3D> pointsInSegment = new ArrayList<>();
   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
   
   public SegmentationNodeData(SegmentedImageRawData seedImageSegment)
   {
      id = seedImageSegment.getId();
      labels.add(seedImageSegment.getImageSegmentLabel());
      labelCenters.add(seedImageSegment.getCenter());
      labelNormals.add(seedImageSegment.getNormal());

      normal.set(seedImageSegment.getNormal());
      center.set(seedImageSegment.getCenter());

      pointsInSegment.addAll(seedImageSegment.getPoints());
   }

   public void merge(SegmentedImageRawData fusionDataSegment)
   {
      labels.add(fusionDataSegment.getImageSegmentLabel());
      labelCenters.add(fusionDataSegment.getCenter());
      labelNormals.add(fusionDataSegment.getNormal());

      pointsInSegment.addAll(fusionDataSegment.getPoints());

      if(USE_PCA_TO_UPDATE)
      {
         fusionDataSegment.getPoints().stream().forEach(point -> pca.addPoint(point.getX(), point.getY(), point.getZ()));
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

   public void extend(SegmentedImageRawData fusionDataSegment, double threshold, boolean updateNodeData, double extendingThreshold)
   {
      for (Point3D point : fusionDataSegment.getPoints())
      {
         double distance = distancePlaneToPoint(normal, center, point);
         if (distance < threshold)
         {
            for (Point3D pointInSegment : pointsInSegment)
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
         PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

         pca.clear();
         pointsInSegment.stream().forEach(point -> pca.addPoint(point.getX(), point.getY(), point.getZ()));
         pca.compute();

         pca.getMean(center);
         pca.getThirdVector(normal);

         if (normal.getZ() < 0.0)
            normal.negate();
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

   public Vector3D getNormal()
   {
      return normal;
   }

   public Point3D getCenter()
   {
      return center;
   }

   public List<Point3D> getPointsInSegment()
   {
      return pointsInSegment;
   }

   /**
    * If this segment is big enough (isBigSegment = true), coplanar test is done with the closest label among this segment.
    */
   public boolean isCoplanar(SegmentedImageRawData fusionDataSegment, double threshold, boolean isBigSegment)
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
         nodeDataCenter.set(labelCenters.get(closestLabel));
         nodeDataNormal.set(labelNormals.get(closestLabel));
      }

      double distanceFromSegment = distancePlaneToPoint(fusionDataSegment.getNormal(), fusionDataSegment.getCenter(), nodeDataCenter);
      double distanceToSegment = distancePlaneToPoint(nodeDataNormal, nodeDataCenter, fusionDataSegment.getCenter());

      if (Math.abs(distanceFromSegment) < threshold && Math.abs(distanceToSegment) < threshold)
         return true;
      else
         return false;
   }

   public boolean isParallel(SegmentedImageRawData fusionDataSegment, double threshold)
   {
      if (Math.abs(fusionDataSegment.getNormal().dot(normal)) > threshold)
         return true;
      else
         return false;
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
