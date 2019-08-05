package us.ihmc.robotEnvironmentAwareness.fusion.data;

import us.ihmc.commons.Conversions;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.dataFactory.FusedSuperPixelsFactory;

import java.text.DecimalFormat;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class FusedSuperPixelImageBuffer
{
   private final FusedSuperPixelsFactory fusedSuperPixelsFactory = new FusedSuperPixelsFactory();

   private final AtomicReference<RawSuperPixelImage> superPixelImage;

   private final AtomicReference<SegmentationRawDataFilteringParameters> segmentationRawDataFilteringParameters;
   private final AtomicReference<PlanarRegionPropagationParameters> planarRegionPropagationParameters;
   private final AtomicReference<SuperPixelNormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> enableREA;
   private final AtomicReference<Boolean> runSingleThreaded;

   private final AtomicReference<List<FusedSuperPixelData>> newBuffer = new AtomicReference<>();

   private final Messager reaMessager;
   private final Messager messager;

   public FusedSuperPixelImageBuffer(Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.reaMessager = reaMessager;
      this.messager = messager;

      enableREA = messager.createInput(LidarImageFusionAPI.EnableREA, false);
      runSingleThreaded = messager.createInput(LidarImageFusionAPI.RunSingleThreaded, false);

      superPixelImage = messager.createInput(LidarImageFusionAPI.RawSuperPixelData);

      segmentationRawDataFilteringParameters = messager.createInput(LidarImageFusionAPI.SegmentationRawDataFilteringParameters,
                                                                    new SegmentationRawDataFilteringParameters());
      planarRegionPropagationParameters = messager.createInput(LidarImageFusionAPI.PlanarRegionPropagationParameters, new PlanarRegionPropagationParameters());
      normalEstimationParameters = messager.createInput(LidarImageFusionAPI.SuperPixelNormalEstimationParameters, new SuperPixelNormalEstimationParameters());
   }


   public void clear()
   {
      newBuffer.set(null);
   }

   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            if (!enableREA.get() || runSingleThreaded.get())
            {
               return;
            }

            RawSuperPixelImage newScan = superPixelImage.getAndSet(null);

            if (newScan == null)
               return;

            long runningStartTime = System.nanoTime();

            reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.

            List<FusedSuperPixelData> fusedSuperPixelData = updateNewBuffer(newScan);
            newBuffer.set(fusedSuperPixelData);
            messager.submitMessage(LidarImageFusionAPI.FusedSuperPixelData, fusedSuperPixelData);

            double segmentationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);

            String segmentationTimeMessage = new DecimalFormat("##.###").format(segmentationTime) + "(sec)";
            messager.submitMessage(LidarImageFusionAPI.DataFusingTime, segmentationTimeMessage);
         }
      };
   }

   public void updateNewBuffer()
   {
      newBuffer.set(updateNewBuffer(superPixelImage.getAndSet(null)));
   }

   public List<FusedSuperPixelData> updateNewBuffer(RawSuperPixelImage rawSuperPixelImage)
   {
      fusedSuperPixelsFactory.updateFusionData(rawSuperPixelImage, segmentationRawDataFilteringParameters.get(), planarRegionPropagationParameters.get(),
                                               normalEstimationParameters.get());
      fusedSuperPixelsFactory.initialize();

      if (!fusedSuperPixelsFactory.fuseSimilarRawSuperPixels())
         return null;

      return fusedSuperPixelsFactory.getFusedSuperPixels();
   }

   public List<FusedSuperPixelData> pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

}
