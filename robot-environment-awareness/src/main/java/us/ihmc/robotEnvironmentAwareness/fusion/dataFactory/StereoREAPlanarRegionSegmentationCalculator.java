package us.ihmc.robotEnvironmentAwareness.fusion.dataFactory;

import java.util.*;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotEnvironmentAwareness.fusion.data.FusedSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelImage;
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
   private final AtomicReference<List<FusedSuperPixelData>> latestFusedSuperPixels = new AtomicReference<>(null);
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
      latestFusedSuperPixels.set(null);
      regionsNodeNata.set(null);
   }

   public List<FusedSuperPixelData> getFusedSuperPixels()
   {
      return latestFusedSuperPixels.get();
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawData()
   {
      return regionsNodeNata.get();
   }

   public boolean fuseSimilarRawSuperPixels()
   {
      List<FusedSuperPixelData> fusedSuperPixels = new ArrayList<>();
      RawSuperPixelImage rawSuperPixelImage = latestSuperPixelImage.get();
      for (int newSegmentId = 0; newSegmentId < NUMBER_OF_ITERATIONS; newSegmentId++)
      {
         if (!growFusedPixelsFromImage(newSegmentId, rawSuperPixelImage, fusedSuperPixels))
            break;
      }

      if (planarRegionPropagationParameters.isExtendingEnabled())
      {
         extendSuperPixelsToIncludeAdjacentUnassignedData(fusedSuperPixels, rawSuperPixelImage, planarRegionPropagationParameters, normalEstimationParameters);
      }

      latestFusedSuperPixels.set(fusedSuperPixels);

      return true;
   }

   public boolean calculatePlanarRegionSegmentationFromSuperPixels()
   {
      regionsNodeNata.set(convertSuperPixelsToPlanarRegionSegmentationRawData(latestFusedSuperPixels.get()));
      return true;
   }

   private boolean growFusedPixelsFromImage(int newSegmentId, RawSuperPixelImage rawSuperPixelImage, List<FusedSuperPixelData> fusedSuperPixelDataToPack)
   {
      RawSuperPixelData unassignedSuperPixel = selectRandomUnassignedPixel(rawSuperPixelImage);
      if (unassignedSuperPixel == null)
         return false;

      FusedSuperPixelData segmentNodeData = createSegmentNodeData(unassignedSuperPixel, newSegmentId, rawSuperPixelImage, planarRegionPropagationParameters,
                                                                  normalEstimationParameters);
      if (segmentNodeData != null)
         fusedSuperPixelDataToPack.add(segmentNodeData);

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
                                                            PlanarRegionPropagationParameters planarRegionPropagationParameters,
                                                            SuperPixelNormalEstimationParameters normalEstimationParameters)
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
            if (checkAndMergeAdjacentSuperPixelsIfValid(newFusedSuperPixel, isBigSegment, rawSuperPixelImage.getSuperPixelData(adjacentPixelLabel),
                                                        planarRegionPropagationParameters, normalEstimationParameters))
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

      newFusedSuperPixel.updateNormal(normalEstimationParameters);

      return newFusedSuperPixel;
   }

   private static boolean checkAndMergeAdjacentSuperPixelsIfValid(FusedSuperPixelData fusedSuperPixelToPack, boolean isBigPixel, RawSuperPixelData candidateAdjacentPixel,
                                                                  PlanarRegionPropagationParameters planarRegionPropagationParameters,
                                                                  SuperPixelNormalEstimationParameters normalEstimationParameters)
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
         fusedSuperPixelToPack.merge(candidateAdjacentPixel, normalEstimationParameters);
         return  true;
      }

      return false;
   }


   private static void extendSuperPixelsToIncludeAdjacentUnassignedData(List<FusedSuperPixelData> fusedSuperPixels, RawSuperPixelImage rawSuperPixelImage,
                                                                        PlanarRegionPropagationParameters planarRegionPropagationParameters,
                                                                        SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      fusedSuperPixels.forEach(fusedSuperPixel -> extendSuperPixelToIncludeAdjacentUnassignedData(fusedSuperPixel, rawSuperPixelImage,
                                                                                                  planarRegionPropagationParameters, normalEstimationParameters));
   }

   private static void extendSuperPixelToIncludeAdjacentUnassignedData(FusedSuperPixelData fusedSuperPixel, RawSuperPixelImage rawSuperPixelImage,
                                                                       PlanarRegionPropagationParameters planarRegionPropagationParameters,
                                                                       SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      Set<Integer> adjacentPixelsInNewImage = rawSuperPixelImage.getLabelsOfSuperPixelsAdjacentToOtherSuperPixels(fusedSuperPixel.getComponentPixelLabels());
      for (int adjacentLabel : adjacentPixelsInNewImage)
      {
         RawSuperPixelData adjacentData = rawSuperPixelImage.getSuperPixelData(adjacentLabel);
         if (adjacentData.getId() == RawSuperPixelData.DEFAULT_SEGMENT_ID)
         {
            SuperPixelTools.extendFusedSuperPixel(fusedSuperPixel, adjacentData, planarRegionPropagationParameters, normalEstimationParameters);
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
