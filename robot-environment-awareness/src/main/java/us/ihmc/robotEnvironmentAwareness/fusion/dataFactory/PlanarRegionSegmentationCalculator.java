package us.ihmc.robotEnvironmentAwareness.fusion.dataFactory;

import us.ihmc.robotEnvironmentAwareness.fusion.data.FusedSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

import java.util.List;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class PlanarRegionSegmentationCalculator
{
   private static final int MINIMUM_NUMBER_OF_SEGMENTATION_RAW_DATA_FOR_PLANAR_REGION = 3;

   private final AtomicReference<List<FusedSuperPixelData>> latestFusedSuperPixels = new AtomicReference<>(null);
   private final AtomicReference<List<PlanarRegionSegmentationRawData>> regionsNodeNata = new AtomicReference<>(null);

   public void updatedFusedSuperPixelData(List<FusedSuperPixelData> fusedSuperPixels)
   {
      latestFusedSuperPixels.set(fusedSuperPixels);
   }

   public void initialize()
   {
      regionsNodeNata.set(null);
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawData()
   {
      return regionsNodeNata.get();
   }

   public boolean convertSuperPixelsToPlanarRegionSegmentationRawData()
   {
      regionsNodeNata.set(convertSuperPixelsToPlanarRegionSegmentationRawData(latestFusedSuperPixels.get()));
      return true;
   }


   /**
    * The id of the PlanarRegionSegmentationRawData is randomly selected to be visualized efficiently rather than selected by SegmentationNodeData.getId().
    */
   private static List<PlanarRegionSegmentationRawData> convertSuperPixelsToPlanarRegionSegmentationRawData(List<FusedSuperPixelData> fusedSuperPixels)
   {
      return fusedSuperPixels.parallelStream().filter(PlanarRegionSegmentationCalculator::hasEnoughComponents)
                             .map(PlanarRegionSegmentationCalculator::convertSuperPixelToPlanarRegionSegmentationRawData).collect(Collectors.toList());
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
