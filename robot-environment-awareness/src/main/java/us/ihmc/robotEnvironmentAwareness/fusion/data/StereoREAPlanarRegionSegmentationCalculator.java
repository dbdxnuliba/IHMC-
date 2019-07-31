package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class StereoREAPlanarRegionSegmentationCalculator
{
   private PlanarRegionPropagationParameters planarRegionPropagationParameters = new PlanarRegionPropagationParameters();
   private SuperPixelNormalEstimationParameters normalEstimationParameters = new SuperPixelNormalEstimationParameters();
   private SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters = new SegmentationRawDataFilteringParameters();

   private static final int NUMBER_OF_ITERATE = 1000;
   private static final int MAXIMUM_NUMBER_OF_TRIALS_TO_FIND_UN_ID_LABEL = 500;
   private static final int MINIMAM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGIEON = 3;
   private static final int MINIMUM_NUMBER_OF_LABELS_FOR_BIG_SEGMENT = 7;

   private final AtomicReference<FusedSuperPixelImage> data = new AtomicReference<FusedSuperPixelImage>(null);
   private int numberOfLabels = 0;
   private final List<SegmentationNodeData> segments = new ArrayList<SegmentationNodeData>();
   private List<PlanarRegionSegmentationRawData> regionsNodeData = new ArrayList<>();

   private final Random random = new Random(0612L);

   public void updateFusionData(FusedSuperPixelImage lidarImageFusionData, SegmentationRawDataFilteringParameters rawDataFilteringParameters,
                                PlanarRegionPropagationParameters propagationParameters, SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      lidarImageFusionData.updateSparsity(rawDataFilteringParameters);
      data.set(lidarImageFusionData);
      numberOfLabels = lidarImageFusionData.getNumberOfImageSegments();
      planarRegionPropagationParameters.set(propagationParameters);
      segmentationRawDataFilteringParameters.set(rawDataFilteringParameters);
      this.normalEstimationParameters.set(normalEstimationParameters);
   }

   public void initialize()
   {
      segments.clear();
      regionsNodeData.clear();
   }

   public boolean calculate()
   {
      for (int i = 0; i < NUMBER_OF_ITERATE; i++)
      {
         if (!iterateSegmenataionPropagation(i))
         {
            break;
         }
      }

      if (planarRegionPropagationParameters.isEnableExtending())
      {
         extendingSegmentations();
      }

      convertNodeDataToPlanarRegionSegmentationRawData();
      return true;
   }

   private void extendingSegmentations()
   {
      for (SegmentationNodeData segment : segments)
      {
         int[] adjacentLabels = data.get().getAdjacentLabels(segment.getLabels());
         for (int adjacentLabel : adjacentLabels)
         {
            RawSuperPixelData adjacentData = data.get().getFusionDataSegment(adjacentLabel);
            if (adjacentData.getId() == RawSuperPixelData.DEFAULT_SEGMENT_ID)
            {
               segment.extend(adjacentData, planarRegionPropagationParameters.getExtendingDistanceThreshold(),
                              planarRegionPropagationParameters.isUpdateExtendedData(), planarRegionPropagationParameters.getExtendingRadiusThreshold(),
                              normalEstimationParameters);
            }

         }
      }
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawData()
   {
      return regionsNodeData;
   }

   /**
    * The id of the PlanarRegionSegmentationRawData is randomly selected to be visualized efficiently rather than selected by SegmentationNodeData.getId().
    */
   private void convertNodeDataToPlanarRegionSegmentationRawData()
   {
      for (SegmentationNodeData segmentationNodeData : segments)
      {
         if (segmentationNodeData.getLabels().size() < MINIMAM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGIEON)
            continue;
         PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(random.nextInt(),
                                                                                                               segmentationNodeData.getNormal(),
                                                                                                               segmentationNodeData.getCenter(),
                                                                                                               segmentationNodeData.getPointsInSegment());
         regionsNodeData.add(planarRegionSegmentationRawData);
      }
   }

   private boolean iterateSegmenataionPropagation(int segmentId)
   {
      int nonIDLabel = selectRandomNonIdentifiedLabel();

      if (nonIDLabel == RawSuperPixelData.DEFAULT_SEGMENT_ID)
      {
         return false;
      }
      else
      {
         SegmentationNodeData segmentNodeData = createSegmentNodeData(nonIDLabel, segmentId);
         if(segmentNodeData != null)
            segments.add(segmentNodeData);
      }

      return true;
   }

   /**
    * iterate computation until there is no more candidate to try merge.
    */
   private SegmentationNodeData createSegmentNodeData(int seedLabel, int segmentId)
   {
      // TODO figure out how to parallelize this
      RawSuperPixelData seedImageSegment = data.get().getFusionDataSegment(seedLabel);
      seedImageSegment.setId(segmentId);
      SegmentationNodeData newSegment = new SegmentationNodeData(seedImageSegment);

      boolean isPropagating = true;

      TIntArrayList labels = newSegment.getLabels();
      while (isPropagating)
      {
         isPropagating = false;
         boolean isBigSegment = labels.size() > MINIMUM_NUMBER_OF_LABELS_FOR_BIG_SEGMENT;

         int[] adjacentLabels = data.get().getAdjacentLabels(labels);

         for (int adjacentLabel : adjacentLabels)
         {
            RawSuperPixelData candidate = data.get().getFusionDataSegment(adjacentLabel);

            if (checkAndMergeLikeSuperPixels(newSegment, candidate, planarRegionPropagationParameters, normalEstimationParameters, isBigSegment, segmentId))
               isPropagating = true;
         }
      }

      boolean resetSmallNodeData = true;
      if (resetSmallNodeData)
      {
         boolean isSmallNodeData = labels.size() < MINIMAM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGIEON;

         if (isSmallNodeData)
         {
            Arrays.stream(labels.toArray()).parallel().forEach(label -> data.get().getFusionDataSegment(label).setId(RawSuperPixelData.DEFAULT_SEGMENT_ID));

            return null;
         }
      }

      return newSegment;
   }

   private static boolean checkAndMergeLikeSuperPixels(SegmentationNodeData mergedNodeSegment, RawSuperPixelData candidateToCheck,
                                                       PlanarRegionPropagationParameters planarRegionPropagationParameters,
                                                       SuperPixelNormalEstimationParameters normalEstimationParameters, boolean isBigSegment,
                                                       int segmentId)
   {
      if (candidateToCheck.getId() != RawSuperPixelData.DEFAULT_SEGMENT_ID || candidateToCheck.isSparse())
      {
         return false;
      }

      boolean isParallel = false;
      boolean isCoplanar = false;
      if (mergedNodeSegment.isParallel(candidateToCheck, planarRegionPropagationParameters.getPlanarityThreshold()))
         isParallel = true;
      if (mergedNodeSegment.isCoplanar(candidateToCheck, planarRegionPropagationParameters.getProximityThreshold(), isBigSegment))
         isCoplanar = true;

      if (isParallel && isCoplanar)
      {
         candidateToCheck.setId(segmentId);
         mergedNodeSegment.merge(candidateToCheck, normalEstimationParameters);
         return true;
      }

      return false;
   }


   private int selectRandomNonIdentifiedLabel()
   {
      int randomSeedLabel = -1;
      for (int i = 0; i < MAXIMUM_NUMBER_OF_TRIALS_TO_FIND_UN_ID_LABEL; i++)
      {
         randomSeedLabel = random.nextInt(numberOfLabels - 1);
         RawSuperPixelData fusionDataSegment = data.get().getFusionDataSegment(randomSeedLabel);
         if (fusionDataSegment.getId() == RawSuperPixelData.DEFAULT_SEGMENT_ID && !fusionDataSegment.isSparse())
            return randomSeedLabel;
      }
      return -1;
   }
}
