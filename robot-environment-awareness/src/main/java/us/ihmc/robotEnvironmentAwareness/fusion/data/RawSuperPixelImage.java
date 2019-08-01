package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;

/**
 * This data set is to hold a list of SegmentationRawData.
 */
public class RawSuperPixelImage
{
   private final int imageWidth;
   private final int imageHeight;
   private final ArrayList<RawSuperPixelData> rawSuperPixels = new ArrayList<>();

   public RawSuperPixelImage(List<RawSuperPixelData> rawSuperPixels, int imageWidth, int imageHeight)
   {
      this.rawSuperPixels.addAll(rawSuperPixels);
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
   }

   public int getNumberOfImageSegments()
   {
      return rawSuperPixels.size();
   }

   public List<RawSuperPixelData> getSuperPixelData()
   {
      return rawSuperPixels;
   }

   public RawSuperPixelData getSuperPixelData(int label)
   {
      return getSuperPixelData().get(label);
   }

   public int[] getAdjacentLabels(TIntArrayList labels)
   {
      TIntArrayList uncompressedAdjacentLabels = new TIntArrayList();
      TIntArrayList adjacentLabels = new TIntArrayList();

      for (int label : labels.toArray())
      {
         uncompressedAdjacentLabels.addAll(rawSuperPixels.get(label).getAdjacentPixelLabels());
      }

      for (int label : uncompressedAdjacentLabels.toArray())
      {
         if (!labels.contains(label) && !adjacentLabels.contains(label))
            adjacentLabels.add(label);
      }

      return adjacentLabels.toArray();
   }

   public boolean allIdentified()
   {
      for (RawSuperPixelData fusionDataSegment : rawSuperPixels)
      {
         if (fusionDataSegment.getId() == -1)
            return false;
      }
      return true;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }
}
