package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This data set includes points which are in a superpixel.
 * The data has its own id and basic planar region information such as center and normal.
 * The adjacent score is to determine which segments are adjacent sufficiently.
 */
public class RawSuperPixelData implements SuperPixel
{
   private static final boolean useParallelStreamsForFiltering = true;

   public static final int DEFAULT_SEGMENT_ID = -1;
   private int id = DEFAULT_SEGMENT_ID;

   private final int imageSegmentLabel;
   private final TIntArrayList adjacentSegmentLabels = new TIntArrayList();
   private final List<Point3D> points = new ArrayList<>();

   private final Point2D segmentCenterInImage = new Point2D();

   private final Point3D center = new Point3D();
   private final Vector3D normal = new Vector3D();

   private final Vector3D standardDeviationVector = new Vector3D();
   private double standardDeviation = Double.NaN;

   private boolean isSparse = true;

   private static final boolean useAdjacentScore = true;
   private static final int numberOfAdjacentPixels = 10;
   private final TIntArrayList adjacentScore = new TIntArrayList();

   public RawSuperPixelData(int labelID)
   {
      imageSegmentLabel = labelID;
      standardDeviationVector.setToNaN();
   }

   public boolean contains(int otherLabel)
   {
      if (useAdjacentScore)
      {
         for (int i = 0; i < adjacentSegmentLabels.size(); i++)
         {
            if (adjacentSegmentLabels.get(i) == otherLabel)
            {
               adjacentScore.replace(i, adjacentScore.get(i) + 1);
               return true;
            }
         }
         return false;
      }
      else
      {
         return adjacentSegmentLabels.contains(otherLabel);
      }
   }

   public void addAdjacentSegmentLabel(int otherLabel)
   {
      adjacentSegmentLabels.add(otherLabel);
      adjacentScore.add(1);
   }

   public void addPoint(Point3D point)
   {
      points.add(point);
   }

   public void filterOutFlyingPoints(double maxDistanceToNeighbor, int minimumNumberOfNeighbors)
   {
      List<Point3D> filteredPoints = new ArrayList<>();
      for (Point3D point : points)
      {
         if (isPointCloseEnoughToOtherPoints(maxDistanceToNeighbor, minimumNumberOfNeighbors, point, points))
            filteredPoints.add(point);
      }
      points.clear();
      points.addAll(filteredPoints);
   }

   private static boolean isPointCloseEnoughToOtherPoints(double minDistanceToNeighbor, int requiredNumberOfNeighbors, Point3D point, List<Point3D> otherPoints)
   {
      requiredNumberOfNeighbors = Math.max(requiredNumberOfNeighbors, 0);
      /*
      int numberOfNeighbors = 0;
      for (Point3D otherPoint : otherPoints)
      {
         if (point == otherPoint)
            continue;

         if (point.distance(otherPoint) < minDistanceToNeighbor)
            numberOfNeighbors++;

         if (numberOfNeighbors > requiredNumberOfNeighbors)
            return true;
      }
      return false;
      */

      Stream<Point3D> pointStream = useParallelStreamsForFiltering ? otherPoints.parallelStream() : otherPoints.stream();
      // subtract 1 to remove self
      int numberOfNeighbors = ((int) pointStream.filter(otherPoint -> point.distance(otherPoint) < minDistanceToNeighbor).count()) - 1;

      return numberOfNeighbors > requiredNumberOfNeighbors;
   }

   public void updateAdjacency()
   {
      if (useAdjacentScore)
      {
         int i = 0;
         while (i < adjacentSegmentLabels.size())
         {
            if (adjacentScore.get(i) > numberOfAdjacentPixels)
            {
               i++;
            }
            else
            {
               adjacentSegmentLabels.remove(i);
               adjacentScore.remove(i);
            }
         }
      }
   }

   public void updateSparsity(double threshold)
   {
      if (!standardDeviationVector.containsNaN())
         isSparse = standardDeviationVector.getZ() > threshold;
      else if (Double.isFinite(standardDeviation) && standardDeviation > threshold)
         isSparse = true;
   }

   /**
    * Not to be sparse,
    * The number of points inside the area within a radius from the center should be over the ratio to all points in segment.
    * @param radius
    * @param threshold
    */
   public void filteringCentrality(double radius, double threshold)
   {
      if (!isSparse)
      {
         Stream<Point3D> pointStream = useParallelStreamsForFiltering ? points.parallelStream() : points.stream();
         if (pointStream.filter(point -> center.distance(point) < radius).count() < threshold * getWeight())
            isSparse = true;

//         int numberOfInliers = 0;
//         for (Point3D point : points)
//         {
//            double distance = center.distance(point);
//            if (distance < radius)
//               numberOfInliers++;
//         }
//
//         if (numberOfInliers < threshold * getWeight())
//         {
//            isSparse = true;
//         }
      }
   }

   /**
    * Not to be sparse,
    * Secondary axis should be over the minLength.
    * Secondary axis should be within threshold * primary axis length.
    * @param minLength
    * @param threshold
    */
   public void filteringEllipticity(double minLength, double threshold)
   {
      if (!isSparse && !standardDeviationVector.containsNaN())
      {
         double lengthPrimary = standardDeviationVector.getX();
         double lengthSecondary = standardDeviationVector.getY();
         if (lengthSecondary < minLength || lengthSecondary > lengthPrimary * threshold)
         {
            isSparse = true;
         }
      }
   }

   public void setId(int id)
   {
      this.id = id;
   }

   public void setSegmentCenter(int u, int v)
   {
      segmentCenterInImage.set(u, v);
   }

   public Point2DReadOnly getSegmentCenter()
   {
      return segmentCenterInImage;
   }

   public boolean isSparse()
   {
      return isSparse;
   }

   public int[] getAdjacentSegmentLabels()
   {
      return adjacentSegmentLabels.toArray();
   }

   public double getWeight()
   {
      return (double) points.size();
   }

   public void setCenter(Point3DReadOnly center)
   {
      this.center.set(center);
   }

   public void setNormal(Vector3DReadOnly normal)
   {
      this.normal.set(normal);
   }

   public void setStandardDeviation(Vector3DReadOnly standardDeviation)
   {
      this.standardDeviationVector.set(standardDeviation);
   }

   public Point3DReadOnly getCenter()
   {
      return center;
   }

   public Vector3DReadOnly getNormal()
   {
      return normal;
   }

   public int getId()
   {
      return id;
   }

   public int getImageSegmentLabel()
   {
      return imageSegmentLabel;
   }

   public List<Point3D> getPoints()
   {
      return points;
   }
}
