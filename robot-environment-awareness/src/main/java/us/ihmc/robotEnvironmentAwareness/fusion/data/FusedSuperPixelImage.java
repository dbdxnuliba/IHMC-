package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;

/**
 * This data set is to hold a list of SegmentationRawData.
 */
public class FusedSuperPixelImage
{
   private static final boolean filterInParallel = true;
   private final int imageWidth;
   private final int imageHeight;
   private final ArrayList<RawSuperPixelData> fusionDataSegments = new ArrayList<RawSuperPixelData>();

   public FusedSuperPixelImage(List<RawSuperPixelData> fusionDataSegments, int imageWidth, int imageHeight)
   {
      this.fusionDataSegments.addAll(fusionDataSegments);
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
   }

   public int getNumberOfImageSegments()
   {
      return fusionDataSegments.size();
   }

   public RawSuperPixelData getFusionDataSegment(int label)
   {
      return fusionDataSegments.get(label);
   }

   public int[] getAdjacentLabels(TIntArrayList labels)
   {
      TIntArrayList uncompressedAdjacentLabels = new TIntArrayList();
      TIntArrayList adjacentLabels = new TIntArrayList();

      for (int label : labels.toArray())
      {
         uncompressedAdjacentLabels.addAll(fusionDataSegments.get(label).getAdjacentSegmentLabels());
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
      for (RawSuperPixelData fusionDataSegment : fusionDataSegments)
      {
         if (fusionDataSegment.getId() == -1)
            return false;
      }
      return true;
   }

   /**
    * Scaled threshold is used according to the v value of the segment center.
    */
   public void updateSparsity(SegmentationRawDataFilteringParameters rawDataFilteringParameters)
   {
      Stream<RawSuperPixelData> superPixelStream = filterInParallel ? fusionDataSegments.parallelStream() : fusionDataSegments.stream();
      superPixelStream.forEach(superPixelData -> updateSparsity(superPixelData, rawDataFilteringParameters, imageHeight));
   }

   private static void updateSparsity(RawSuperPixelData superPixelData, SegmentationRawDataFilteringParameters rawDataFilteringParameters, int imageHeight)
   {
      double sparseLowerThreshold = rawDataFilteringParameters.getMinimumSparseThreshold();
      double sparseUpperThreshold = sparseLowerThreshold * rawDataFilteringParameters.getMaximumSparsePropotionalRatio();

      double alpha = 1 - superPixelData.getSegmentCenter().getY() / imageHeight;
      double threshold = alpha * (sparseUpperThreshold - sparseLowerThreshold) + sparseLowerThreshold;

      superPixelData.updateSparsity(threshold);
      superPixelData.updateSparsityFromNormalQuality(rawDataFilteringParameters.getMinNormalVariance(), rawDataFilteringParameters.getMinNormalConsensus());

      if (rawDataFilteringParameters.isEnableFilterCentrality())
         superPixelData.filteringCentrality(rawDataFilteringParameters.getCentralityRadius(), rawDataFilteringParameters.getCentralityThreshold());

      if (rawDataFilteringParameters.isEnableFilterEllipticity())
         superPixelData.filteringEllipticity(rawDataFilteringParameters.getEllipticityMinimumLength(), rawDataFilteringParameters.getEllipticityThreshold());
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
