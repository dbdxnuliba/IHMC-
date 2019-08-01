package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;

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
public class RawSuperPixelData implements SuperPixelData
{
   public static final int DEFAULT_SEGMENT_ID = -1;
   private int id = DEFAULT_SEGMENT_ID;

   private final int imageSegmentLabel;
   private final List<AdjacentPixelData> adjacentPixelsData = new ArrayList<>();
   private final List<Point3DReadOnly> points = new ArrayList<>();

   private final Point2D segmentCenterInImage = new Point2D();

   private final Point3D center = new Point3D();
   private final Vector3D normal = new Vector3D();

   private final Vector3D standardDeviation = new Vector3D();
   private double normalVariance = Double.NaN;
   private int normalConsensus = Integer.MIN_VALUE;

   private boolean isSparse = false;

   private static final boolean useAdjacentScore = true;
   private static final int minimumNumberOfAdjacentPixels = 10;

   public RawSuperPixelData(int labelID)
   {
      imageSegmentLabel = labelID;

      center.setToNaN();
      normal.setToNaN();
      standardDeviation.setToNaN();
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

   public Vector3DReadOnly getStandardDeviation()
   {
      return standardDeviation;
   }

   @Override
   public List<Point3DReadOnly> getPointsInPixel()
   {
      return points;
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

   public boolean hasStandardDeviation()
   {
      return !standardDeviation.containsNaN();
   }

   public boolean contains(int otherLabel)
   {
      for (int i = 0; i < adjacentPixelsData.size(); i++)
      {
         if (adjacentPixelsData.get(i).pixelId == otherLabel)
         {
            if (useAdjacentScore)
               adjacentPixelsData.get(i).incrementScore();
            return true;
         }
      }

      return false;
   }

   public void addAdjacentPixel(int adjacentPixelLabel)
   {
      adjacentPixelsData.add(new AdjacentPixelData(adjacentPixelLabel, 1));
   }

   public void addPoint(Point3DReadOnly point)
   {
      points.add(point);
   }

   public void updateAdjacency()
   {
      if (useAdjacentScore)
      {
         int i = 0;
         while (i < adjacentPixelsData.size())
         {
            if (adjacentPixelsData.get(i).getNumberOfAdjacentPixels() > minimumNumberOfAdjacentPixels)
               i++;
            else
               adjacentPixelsData.remove(i);
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

   public void setIsSparse(boolean isSparse)
   {
      this.isSparse = isSparse;
   }

   public int[] getAdjacentPixelLabels()
   {
      int[] labels = new int[adjacentPixelsData.size()];
      for (int i = 0; i < adjacentPixelsData.size(); i++)
         labels[i] = adjacentPixelsData.get(i).getPixelId();
      return labels;
   }

   public double getWeight()
   {
      return (double) points.size();
   }

   public int getId()
   {
      return id;
   }

   public int getImageSegmentLabel()
   {
      return imageSegmentLabel;
   }


   private class AdjacentPixelData
   {
      private final int pixelId;
      private int numberOfAdjacentPixels;

      public AdjacentPixelData(int pixelId, int numberOfAdjacentPixels)
      {
         this.pixelId = pixelId;
         this.numberOfAdjacentPixels = numberOfAdjacentPixels;
      }

      public void incrementScore()
      {
         numberOfAdjacentPixels++;
      }

      public int getNumberOfAdjacentPixels()
      {
         return numberOfAdjacentPixels;
      }

      public int getPixelId()
      {
         return pixelId;
      }
   }
}
