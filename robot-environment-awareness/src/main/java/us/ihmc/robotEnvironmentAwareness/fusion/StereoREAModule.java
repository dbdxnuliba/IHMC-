package us.ihmc.robotEnvironmentAwareness.fusion;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.*;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

public class StereoREAModule
{
   private final Messager reaMessager;
   private final Messager messager;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> isRunning = new AtomicReference<Boolean>(false);
   private final RawSuperPixelImageBuffer rawSuperPixelImageBuffer;
   private final FusedSuperPixelImageBuffer fusedSuperPixelImageBuffer;
   private final PlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProvider;

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 200;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);

   private ScheduledFuture<?> scheduled;

   public StereoREAModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.reaMessager = reaMessager;
      rawSuperPixelImageBuffer = new RawSuperPixelImageBuffer(messager, PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters);
      fusedSuperPixelImageBuffer = new FusedSuperPixelImageBuffer(reaMessager, messager);
      planarRegionFeatureUpdater = new PlanarRegionFeatureUpdater(reaMessager, messager);

      enable = messager.createInput(LidarImageFusionAPI.EnableREA, false);

      planarRegionNetworkProvider = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdater, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);

      initializeREAPlanarRegionPublicNetworkProvider();
   }

   private void initializeREAPlanarRegionPublicNetworkProvider()
   {
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeClear, false);
      reaMessager.submitMessage(REAModuleAPI.LidarMinRange, Double.NEGATIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, Double.POSITIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, new BoundingBoxParametersMessage());
   }

   public void registerCustomPlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionFeatureUpdater.registerCustomPlanarRegion(planarRegion);
   }

   public void dispatchCustomPlanarRegion(PlanarRegionsListMessage message)
   {
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      rawSuperPixelImageBuffer.updateLatestStereoVisionPointCloudMessage(message);
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
      rawSuperPixelImageBuffer.updateLatestBufferedImage(bufferedImage);
   }

   public void start()
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(rawSuperPixelImageBuffer.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(fusedSuperPixelImageBuffer.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(planarRegionFeatureUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      LogTools.info("REA Module is going down.");

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   public void mainUpdate()
   {
      if (enable.get())
      {
         isRunning.set(true);

         planarRegionNetworkProvider.update(true);
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true);
         planarRegionNetworkProvider.publishCurrentState();
      }
      else
      {
         isRunning.set(false);
      }
   }

   public void singleRun()
   {
      isRunning.set(true);
      long runningStartTime = System.nanoTime();

      rawSuperPixelImageBuffer.updateNewBuffer();

      RawSuperPixelImage rawSuperPixelData = rawSuperPixelImageBuffer.pollNewBuffer();
      messager.submitMessage(LidarImageFusionAPI.RawSuperPixelData, rawSuperPixelData);

      double filteringTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);

      fusedSuperPixelImageBuffer.updateNewBuffer();

      List<FusedSuperPixelData> fusedSuperPixelData = fusedSuperPixelImageBuffer.pollNewBuffer();
      messager.submitMessage(LidarImageFusionAPI.FusedSuperPixelData, fusedSuperPixelData);

      double fusingTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime) - filteringTime;

      planarRegionFeatureUpdater.updateLatestSuperPixels(fusedSuperPixelData);

      if (planarRegionFeatureUpdater.update())
      {
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
         reportPlanarRegionState();
      }

      double segmentationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime) - (fusingTime + filteringTime);


      double runningTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);
      String filteringTimeMessage = new DecimalFormat("##.###").format(filteringTime) + "(sec)";
      String fusingTimeMessage = new DecimalFormat("##.###").format(fusingTime) + "(sec)";
      String segmentationTimeMessage = new DecimalFormat("##.###").format(segmentationTime) + "(sec)";
      String runningTimeMessage = new DecimalFormat("##.###").format(runningTime) + "(sec)";

      messager.submitMessage(LidarImageFusionAPI.DataFilteringTime, filteringTimeMessage);
      messager.submitMessage(LidarImageFusionAPI.DataFusingTime, fusingTimeMessage);
      messager.submitMessage(LidarImageFusionAPI.ImageSegmentationTime, segmentationTimeMessage);
      messager.submitMessage(LidarImageFusionAPI.ComputationTime, runningTimeMessage);
      isRunning.set(false);
   }

   public void enable()
   {
      enable.set(true);
   }

   private void reportPlanarRegionState()
   {
      if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
      {
         PlanarRegionsList planarRegionsList = planarRegionFeatureUpdater.getPlanarRegionsList();
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);

         planarRegionNetworkProvider.update(true);
         planarRegionNetworkProvider.publishCurrentState();
      }
   }
}
