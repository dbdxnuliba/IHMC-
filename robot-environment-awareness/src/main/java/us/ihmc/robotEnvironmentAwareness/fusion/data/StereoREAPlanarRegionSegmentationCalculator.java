package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.*;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SegmentationRawDataFiltering;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SuperPixelTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class StereoREAPlanarRegionSegmentationCalculator
{
   private final PlanarRegionPropagationParameters planarRegionPropagationParameters = new PlanarRegionPropagationParameters();
   private final SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters = new SegmentationRawDataFilteringParameters();
   private final SuperPixelNormalEstimationParameters normalEstimationParameters = new SuperPixelNormalEstimationParameters();

   private static final int NUMBER_OF_ITERATIONS = 1000;
   private static final int MAXIMUM_NUMBER_OF_TRIALS_TO_FIND_UN_ID_LABEL = 500;
   private static final int MINIMUM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGION = 3;
   private static final int MINIMUM_NUMBER_OF_LABELS_FOR_BIG_SEGMENT = 7;

   private static final boolean resetSmallNodeData = true;

   private final AtomicReference<RawSuperPixelImage> latestSuperPixelImage = new AtomicReference<>(null);
   private final List<FusedSuperPixelData> fusedSuperPixels = new ArrayList<>();
   private final AtomicReference<List<PlanarRegionSegmentationRawData>> regionsNodeNata = new AtomicReference<>(null);

   public void updateFusionData(RawSuperPixelImage rawSuperPixelImage, SegmentationRawDataFilteringParameters rawDataFilteringParameters,
                                PlanarRegionPropagationParameters propagationParameters, SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      SegmentationRawDataFiltering.updateSparsity(rawSuperPixelImage, rawDataFilteringParameters);
      latestSuperPixelImage.set(rawSuperPixelImage);
      planarRegionPropagationParameters.set(propagationParameters);
      segmentationRawDataFilteringParameters.set(rawDataFilteringParameters);
      this.normalEstimationParameters.set(normalEstimationParameters);
   }

   public void initialize()
   {
      fusedSuperPixels.clear();
      regionsNodeNata.set(null);
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawData()
   {
      return regionsNodeNata.get();
   }

   public boolean calculate()
   {
      RawSuperPixelImage rawSuperPixelImage = latestSuperPixelImage.get();
      for (int newSegmentId = 0; newSegmentId < NUMBER_OF_ITERATIONS; newSegmentId++)
      {
         if (!growFusedPixelsFromImage(newSegmentId, rawSuperPixelImage))
            break;
      }

      if (planarRegionPropagationParameters.isExtendingEnabled())
      {
         extendSuperPixelsToIncludeAdjacentUnassignedData(fusedSuperPixels, rawSuperPixelImage, planarRegionPropagationParameters);
      }

      regionsNodeNata.set(convertSuperPixelsToPlanarRegionSegmentationRawData(fusedSuperPixels));
      return true;
   }

   private boolean growFusedPixelsFromImage(int newSegmentId, RawSuperPixelImage rawSuperPixelImage)
   {
      RawSuperPixelData unassignedSuperPixel = selectRandomUnassignedPixel(rawSuperPixelImage);
      if (unassignedSuperPixel == null)
         return false;

      FusedSuperPixelData segmentNodeData = createSegmentNodeData(unassignedSuperPixel, newSegmentId, rawSuperPixelImage, planarRegionPropagationParameters);
      if (segmentNodeData != null)
         fusedSuperPixels.add(segmentNodeData);

      return true;
   }

   private static RawSuperPixelData selectRandomUnassignedPixel(RawSuperPixelImage rawSuperPixelImage)
   {
      for (int i = 0; i < MAXIMUM_NUMBER_OF_TRIALS_TO_FIND_UN_ID_LABEL; i++)
      {
         int randomSeedLabel = ThreadLocalRandom.current().nextInt(rawSuperPixelImage.getNumberOfImageSegments() - 1);

         RawSuperPixelData superPixel = rawSuperPixelImage.getSuperPixelData(randomSeedLabel);

         if (superPixel.getId() == RawSuperPixelData.DEFAULT_SEGMENT_ID && !superPixel.isSparse())
            return superPixel;
      }

      return null;
   }

   /**
    * iterate computation until there is no more candidate to try merge.
    */
   private static FusedSuperPixelData createSegmentNodeData(RawSuperPixelData unassignedSuperPixel, int segmentId, RawSuperPixelImage rawSuperPixelImage,
                                                            PlanarRegionPropagationParameters planarRegionPropagationParameters)
   {
      unassignedSuperPixel.setId(segmentId);
      FusedSuperPixelData newFusedSuperPixel = new FusedSuperPixelData(unassignedSuperPixel);

      boolean isPropagating = true;

      TIntArrayList labelsAlreadyInFusedSuperPixel = newFusedSuperPixel.getComponentPixelLabels();
      while (isPropagating)
      {
         isPropagating = false;
         boolean isBigSegment = labelsAlreadyInFusedSuperPixel.size() > MINIMUM_NUMBER_OF_LABELS_FOR_BIG_SEGMENT;

         Set<Integer> adjacentPixelLabels = rawSuperPixelImage.getLabelsOfSuperPixelsAdjacentToOtherSuperPixels(labelsAlreadyInFusedSuperPixel);

         for (int adjacentPixelLabel : adjacentPixelLabels)
         {
            if (checkAndMergeAdjacentSuperPixelsIfValid(newFusedSuperPixel, isBigSegment, rawSuperPixelImage.getSuperPixelData(adjacentPixelLabel), planarRegionPropagationParameters))
               isPropagating = true;
         }
      }

      if (resetSmallNodeData)
      {
         boolean isSmallNodeData = labelsAlreadyInFusedSuperPixel.size() < MINIMUM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGION;

         if (isSmallNodeData)
         {
            Arrays.stream(labelsAlreadyInFusedSuperPixel.toArray()).forEach(label -> rawSuperPixelImage.getSuperPixelData(label).setId(RawSuperPixelData.DEFAULT_SEGMENT_ID));

            return null;
         }
      }

      return newFusedSuperPixel;
   }

   private static boolean checkAndMergeAdjacentSuperPixelsIfValid(FusedSuperPixelData fusedSuperPixelToPack, boolean isBigPixel, RawSuperPixelData candidateAdjacentPixel,
                                                                  PlanarRegionPropagationParameters planarRegionPropagationParameters)
   {
      // skipped this pixel if it's already been assigned, or it's sparse
      if (candidateAdjacentPixel.getId() != RawSuperPixelData.DEFAULT_SEGMENT_ID || candidateAdjacentPixel.isSparse())
      {
         return false;
      }

      boolean isParallel = SuperPixelTools.areSuperPixelsParallel(fusedSuperPixelToPack, candidateAdjacentPixel, planarRegionPropagationParameters.getPlanarityThreshold());
      boolean isCoplanar = SuperPixelTools.areSuperPixelsCoplanar(fusedSuperPixelToPack, isBigPixel, candidateAdjacentPixel, planarRegionPropagationParameters.getProximityThreshold());

      if (isParallel && isCoplanar)
      {
         candidateAdjacentPixel.setId(fusedSuperPixelToPack.getId());
         fusedSuperPixelToPack.merge(candidateAdjacentPixel);
         return  true;
      }

      return false;
   }


   private static void extendSuperPixelsToIncludeAdjacentUnassignedData(List<FusedSuperPixelData> fusedSuperPixels, RawSuperPixelImage rawSuperPixelImage,
                                                                        PlanarRegionPropagationParameters planarRegionPropagationParameters)
   {
      fusedSuperPixels.parallelStream().forEach(fusedSuperPixel -> extendSuperPixelToIncludeAdjacentUnassignedData(fusedSuperPixel, rawSuperPixelImage,
                                                                                                                   planarRegionPropagationParameters));
   }

   private static void extendSuperPixelToIncludeAdjacentUnassignedData(FusedSuperPixelData fusedSuperPixel, RawSuperPixelImage rawSuperPixelImage,
                                                                       PlanarRegionPropagationParameters planarRegionPropagationParameters)
   {
      Set<Integer> adjacentPixelsInNewImage = rawSuperPixelImage.getLabelsOfSuperPixelsAdjacentToOtherSuperPixels(fusedSuperPixel.getComponentPixelLabels());
      for (int adjacentLabel : adjacentPixelsInNewImage)
      {
         RawSuperPixelData adjacentData = rawSuperPixelImage.getSuperPixelData(adjacentLabel);
         if (adjacentData.getId() == RawSuperPixelData.DEFAULT_SEGMENT_ID)
         {
            SuperPixelTools.extendFusedSuperPixel(fusedSuperPixel, adjacentData, planarRegionPropagationParameters);
         }
      }
   }

   /**
    * The id of the PlanarRegionSegmentationRawData is randomly selected to be visualized efficiently rather than selected by SegmentationNodeData.getId().
    */
   private static List<PlanarRegionSegmentationRawData> convertSuperPixelsToPlanarRegionSegmentationRawData(List<FusedSuperPixelData> fusedSuperPixels)
   {
      return fusedSuperPixels.parallelStream().filter(StereoREAPlanarRegionSegmentationCalculator::hasEnoughComponents)
                      .map(StereoREAPlanarRegionSegmentationCalculator::convertSuperPixelToPlanarRegionSegmentationRawData).collect(Collectors.toList());
   }

   private static boolean hasEnoughComponents(FusedSuperPixelData fusedSuperPixelData)
   {
      return fusedSuperPixelData.getNumberOfComponentSuperPixels() >= MINIMUM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGION;
   }

   private static PlanarRegionSegmentationRawData convertSuperPixelToPlanarRegionSegmentationRawData(FusedSuperPixelData fusedSuperPixelData)
   {
      return new PlanarRegionSegmentationRawData(ThreadLocalRandom.current().nextInt(), fusedSuperPixelData.getNormal(), fusedSuperPixelData.getCenter(),
                                                 fusedSuperPixelData.getPointsInPixel());

   }
}
