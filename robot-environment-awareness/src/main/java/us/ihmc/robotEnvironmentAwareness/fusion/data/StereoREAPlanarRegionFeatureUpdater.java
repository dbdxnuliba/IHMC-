package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.text.DecimalFormat;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class StereoREAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private final StereoREAPlanarRegionSegmentationCalculator planarRegionSegmentationCalculator = new StereoREAPlanarRegionSegmentationCalculator();
   private PlanarRegionsList planarRegionsList = null;

   private final TIntObjectHashMap<PlanarRegion> customPlanarRegions = new TIntObjectHashMap<>();

   private final AtomicReference<RawSuperPixelImage> superPixelImage;
   private final AtomicReference<Boolean> enableCustomRegions;
   private final AtomicReference<Boolean> clearCustomRegions;
   private final AtomicReference<CustomRegionMergeParameters> customRegionMergingParameters;

   private final AtomicReference<SegmentationRawDataFilteringParameters> segmentationRawDataFilteringParameters;
   private final AtomicReference<PlanarRegionPropagationParameters> planarRegionPropagationParameters;
   private final AtomicReference<ConcaveHullFactoryParameters> concaveHullFactoryParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;
   private final AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;

   private final AtomicReference<Boolean> enableREA;

   private List<LineSegment3D> planarRegionsIntersections = null;

   private final Messager reaMessager;
   private final Messager messager;

   public StereoREAPlanarRegionFeatureUpdater(Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.reaMessager = reaMessager;
      this.messager = messager;

      enableCustomRegions = reaMessager.createInput(REAModuleAPI.CustomRegionsMergingEnable, true);
      clearCustomRegions = reaMessager.createInput(REAModuleAPI.CustomRegionsClear, false);

      enableREA = messager.createInput(LidarImageFusionAPI.EnableREA, false);

      superPixelImage = messager.createInput(LidarImageFusionAPI.FusionDataState);

      concaveHullFactoryParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsConcaveHullParameters, new ConcaveHullFactoryParameters());
      polygonizerParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsPolygonizerParameters, new PolygonizerParameters());
      intersectionEstimationParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsIntersectionParameters, new IntersectionEstimationParameters());
      segmentationRawDataFilteringParameters = messager.createInput(LidarImageFusionAPI.SegmentationRawDataFilteringParameters,
                                                                    new SegmentationRawDataFilteringParameters());
      planarRegionPropagationParameters = messager.createInput(LidarImageFusionAPI.PlanarRegionPropagationParameters, new PlanarRegionPropagationParameters());
      customRegionMergingParameters = reaMessager.createInput(REAModuleAPI.CustomRegionsMergingParameters, new CustomRegionMergeParameters());
   }

   public void updateLatestLidarImageFusionData(RawSuperPixelImage rawSuperPixelImage)
   {
      planarRegionSegmentationCalculator.updateFusionData(rawSuperPixelImage, segmentationRawDataFilteringParameters.get(),
                                                          planarRegionPropagationParameters.get());
      planarRegionSegmentationCalculator.initialize();
   }

   public void clear()
   {
      planarRegionsList = null;
   }

   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            if (!enableREA.get())
            {
               return;
            }

            RawSuperPixelImage newScan = superPixelImage.getAndSet(null);

            if (newScan == null)
               return;

            long runningStartTime = System.nanoTime();

            updateLatestLidarImageFusionData(newScan);

            if (update() && planarRegionsList != null)
            {
               reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
               PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
               reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);

               double segmentationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);

               String segmentationTimeMessage = new DecimalFormat("##.###").format(segmentationTime) + "(sec)";
               messager.submitMessage(LidarImageFusionAPI.ImageSegmentationTime, segmentationTimeMessage);
            }
         }
      };
   }

   public boolean update()
   {
      boolean calculationIsDone = planarRegionSegmentationCalculator.calculate();
      if (calculationIsDone)
      {
         List<PlanarRegionSegmentationRawData> rawData = planarRegionSegmentationCalculator.getSegmentationRawData();
         updateIntersections(rawData);
         mergeCustomPlanarRegions(rawData);
         updatePolygons(rawData);

         return true;
      }

      return false;
   }

   private void mergeCustomPlanarRegions(List<PlanarRegionSegmentationRawData> rawData)
   {
      List<PlanarRegion> unmergedCustomPlanarRegions = Collections.emptyList();
      if (clearCustomRegions.getAndSet(false))
      {
         customPlanarRegions.clear();
         unmergedCustomPlanarRegions = Collections.emptyList();
      }
      else if (enableCustomRegions.get())
      {
         unmergedCustomPlanarRegions = CustomPlanarRegionHandler.mergeCustomRegionsToEstimatedRegions(customPlanarRegions.valueCollection(), rawData,
                                                                                                      customRegionMergingParameters.get());
      }
      else
      {
         unmergedCustomPlanarRegions = Collections.emptyList();
      }

      if (planarRegionsList != null)
         unmergedCustomPlanarRegions.forEach(planarRegionsList::addPlanarRegion);
   }

   private void updatePolygons(List<PlanarRegionSegmentationRawData> rawData)
   {
      ConcaveHullFactoryParameters concaveHullFactoryParameters = this.concaveHullFactoryParameters.get();
      PolygonizerParameters polygonizerParameters = this.polygonizerParameters.get();

      planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   private void updateIntersections(List<PlanarRegionSegmentationRawData> rawData)
   {
      planarRegionsIntersections = PlanarRegionIntersectionCalculator.computeIntersections(rawData, intersectionEstimationParameters.get());
   }

   public void registerCustomPlanarRegion(PlanarRegion planarRegion)
   {
      if (planarRegion.getRegionId() == PlanarRegion.NO_REGION_ID)
      {
         return;
      }
      else if (planarRegion.isEmpty())
      {
         customPlanarRegions.remove(planarRegion.getRegionId());
      }
      else
      {
         CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(planarRegion);
         customPlanarRegions.put(planarRegion.getRegionId(), planarRegion);
      }
   }

   @Override
   public List<PlanarRegionSegmentationNodeData> getSegmentationNodeData()
   {
      return null;
   }

   @Override
   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   @Override
   public int getNumberOfPlaneIntersections()
   {
      return planarRegionsIntersections == null ? 0 : planarRegionsIntersections.size();
   }

   @Override
   public LineSegment3D getIntersection(int index)
   {
      return planarRegionsIntersections.get(index);
   }
}
