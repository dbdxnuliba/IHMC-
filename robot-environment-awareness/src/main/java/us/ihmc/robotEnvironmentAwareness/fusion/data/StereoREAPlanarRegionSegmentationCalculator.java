package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SegmentationRawDataFiltering;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class StereoREAPlanarRegionSegmentationCalculator
{
   private final PlanarRegionPropagationParameters planarRegionPropagationParameters = new PlanarRegionPropagationParameters();
   private final SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters = new SegmentationRawDataFilteringParameters();
   private final SuperPixelNormalEstimationParameters normalEstimationParameters = new SuperPixelNormalEstimationParameters();

   private static final int NUMBER_OF_ITERATE = 1000;
   private static final int MAXIMUM_NUMBER_OF_TRIALS_TO_FIND_UN_ID_LABEL = 500;
   private static final int MINIMAM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGIEON = 3;
   private static final int MINIMUM_NUMBER_OF_LABELS_FOR_BIG_SEGMENT = 7;

   private final AtomicReference<RawSuperPixelImage> data = new AtomicReference<RawSuperPixelImage>(null);
   private int numberOfLabels = 0;
   private final List<FusedSuperPixelData> segments = new ArrayList<FusedSuperPixelData>();
   private List<PlanarRegionSegmentationRawData> regionsNodeData = new ArrayList<>();

   private final Random random = new Random(0612L);

   public void updateFusionData(RawSuperPixelImage rawSuperPixelImage, SegmentationRawDataFilteringParameters rawDataFilteringParameters,
                                PlanarRegionPropagationParameters propagationParameters, SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      SegmentationRawDataFiltering.updateSparsity(rawSuperPixelImage, rawDataFilteringParameters);
      data.set(rawSuperPixelImage);
      numberOfLabels = rawSuperPixelImage.getNumberOfImageSegments();
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
      for (FusedSuperPixelData segment : segments)
      {
         int[] adjacentLabels = data.get().getAdjacentLabels(segment.getLabels());
         for (int adjacentLabel : adjacentLabels)
         {
            RawSuperPixelData adjacentData = data.get().getSuperPixelData(adjacentLabel);
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
      for (FusedSuperPixelData fusedSuperPixelData : segments)
      {
         if (fusedSuperPixelData.getLabels().size() < MINIMAM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGIEON)
            continue;
         PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(random.nextInt(),
                                                                                                               fusedSuperPixelData.getNormal(),
                                                                                                               fusedSuperPixelData.getCenter(),
                                                                                                               fusedSuperPixelData.getPointsInPixel());
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
         FusedSuperPixelData segmentNodeData = createSegmentNodeData(nonIDLabel, segmentId);
         if(segmentNodeData != null)
            segments.add(segmentNodeData);
      }

      return true;
   }

   /**
    * iterate computation until there is no more candidate to try merge.
    */
   private FusedSuperPixelData createSegmentNodeData(int seedLabel, int segmentId)
   {
      RawSuperPixelData seedSuperPixel = data.get().getSuperPixelData(seedLabel);
      seedSuperPixel.setId(segmentId);
      FusedSuperPixelData fusedSuperPixel = new FusedSuperPixelData(seedSuperPixel);

      boolean isPropagating = true;

      TIntArrayList labels = fusedSuperPixel.getLabels();
      while (isPropagating)
      {
         isPropagating = false;
         boolean isBigSegment = labels.size() > MINIMUM_NUMBER_OF_LABELS_FOR_BIG_SEGMENT;

         int[] adjacentLabels = data.get().getAdjacentLabels(labels);

         for (int adjacentLabel : adjacentLabels)
         {
            RawSuperPixelData candidate = data.get().getSuperPixelData(adjacentLabel);

            if (candidate.getId() != RawSuperPixelData.DEFAULT_SEGMENT_ID || candidate.isSparse())
            {
               continue;
            }

            boolean isParallel = fusedSuperPixel.isParallel(candidate, planarRegionPropagationParameters.getPlanarityThreshold());
            boolean isCoplanar = fusedSuperPixel.isCoplanar(candidate, planarRegionPropagationParameters.getProximityThreshold(), isBigSegment);

            if (isParallel && isCoplanar)
            {
               candidate.setId(segmentId);
               fusedSuperPixel.merge(candidate);
               isPropagating = true;
            }
         }
      }

      boolean resetSmallNodeData = true;
      if (resetSmallNodeData)
      {
         boolean isSmallNodeData = labels.size() < MINIMAM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGIEON;
         if(isSmallNodeData)
         {
            for(int label : labels.toArray())
            {
               RawSuperPixelData rawData = data.get().getSuperPixelData(label);
               rawData.setId(RawSuperPixelData.DEFAULT_SEGMENT_ID);
            }
            return null;
         }
      }

      return fusedSuperPixel;
   }

   private int selectRandomNonIdentifiedLabel()
   {
      int randomSeedLabel = -1;
      for (int i = 0; i < MAXIMUM_NUMBER_OF_TRIALS_TO_FIND_UN_ID_LABEL; i++)
      {
         randomSeedLabel = random.nextInt(numberOfLabels - 1);
         RawSuperPixelData fusionDataSegment = data.get().getSuperPixelData(randomSeedLabel);
         if (fusionDataSegment.getId() == RawSuperPixelData.DEFAULT_SEGMENT_ID && !fusionDataSegment.isSparse())
            return randomSeedLabel;
      }
      return -1;
   }
}
