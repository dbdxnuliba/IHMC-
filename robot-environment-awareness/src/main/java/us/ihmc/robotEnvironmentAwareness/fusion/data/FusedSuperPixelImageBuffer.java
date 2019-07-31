package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.FusedSuperPixelImageFactory;

public class FusedSuperPixelImageBuffer
{
   private final FusedSuperPixelImageFactory fusedSuperPixelImageFactory = new FusedSuperPixelImageFactory();

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);

   private final AtomicReference<Point3D> latestCameraPosition;
   private final AtomicReference<Quaternion> latestCameraOrientation;
   private final AtomicReference<IntrinsicParameters> latestCameraIntrinsicParameters;

   private final AtomicReference<Boolean> enableREA;

   private final AtomicReference<Integer> bufferSize;

   private final AtomicReference<SegmentationRawDataFilteringParameters> latestSegmentationRawDataFilteringParameters;
   private final AtomicReference<ImageSegmentationParameters> latestImageSegmentationParaeters;
   private final AtomicReference<RawSuperPixelImage> newBuffer = new AtomicReference<>(null);

   private final Messager messager;

   public FusedSuperPixelImageBuffer(Messager messager, IntrinsicParameters intrinsic)
   {
      this.messager = messager;

      //intrinsicParameters = intrinsic;
      bufferSize = messager.createInput(LidarImageFusionAPI.StereoBufferSize, 50000);
      latestImageSegmentationParaeters = messager.createInput(LidarImageFusionAPI.ImageSegmentationParameters, new ImageSegmentationParameters());
      latestSegmentationRawDataFilteringParameters = messager.createInput(LidarImageFusionAPI.SegmentationRawDataFilteringParameters,
                                                                          new SegmentationRawDataFilteringParameters());

      enableREA = messager.createInput(LidarImageFusionAPI.EnableREA, false);


      latestCameraPosition = messager.createInput(LidarImageFusionAPI.CameraPositionState, new Point3D());
      latestCameraOrientation = messager.createInput(LidarImageFusionAPI.CameraOrientationState, new Quaternion());
      latestCameraIntrinsicParameters = messager.createInput(LidarImageFusionAPI.CameraIntrinsicParametersState, new IntrinsicParameters());
   }

   public RawSuperPixelImage pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

   public void updateNewBuffer()
   {
      newBuffer.set(updateNewBuffer(latestStereoVisionPointCloudMessage.get()));
   }

   public RawSuperPixelImage updateNewBuffer(StereoVisionPointCloudMessage pointCloudMessage)
   {
      if (pointCloudMessage == null)
         return null;

      Point3D[] pointCloudBuffer = MessageTools.unpackScanPoint3ds(pointCloudMessage);
      int[] colorBuffer = pointCloudMessage.getColors().toArray();
      Random random = new Random();
      int numberOfPoints = pointCloudBuffer.length;

      while (numberOfPoints > bufferSize.get())
      {
         int indexToRemove = random.nextInt(numberOfPoints);
         int lastIndex = numberOfPoints - 1;

         pointCloudBuffer[indexToRemove] = pointCloudBuffer[lastIndex];
         colorBuffer[indexToRemove] = colorBuffer[lastIndex];

         numberOfPoints--;
      }

      ColoredPixel[] coloredPixels = new ColoredPixel[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         coloredPixels[i] = new ColoredPixel(pointCloudBuffer[i], colorBuffer[i]);
      }

      fusedSuperPixelImageFactory.setIntrinsicParameters(latestCameraIntrinsicParameters.get());
      fusedSuperPixelImageFactory.setImageSegmentationParameters(latestImageSegmentationParaeters.get());
      fusedSuperPixelImageFactory.setSegmentationRawDataFilteringParameters(latestSegmentationRawDataFilteringParameters.get());
      fusedSuperPixelImageFactory.setCameraPose(latestCameraPosition.get(), latestCameraOrientation.get());

      return fusedSuperPixelImageFactory.createRawSuperPixelImage(coloredPixels, latestBufferedImage.get());
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

            StereoVisionPointCloudMessage newScan = latestStereoVisionPointCloudMessage.getAndSet(null);

            if (newScan == null)
               return;

            long startTime = System.nanoTime();

            RawSuperPixelImage superPixelImage = updateNewBuffer(newScan);
            newBuffer.set(superPixelImage);
            messager.submitMessage(LidarImageFusionAPI.FusionDataState, superPixelImage);

            double runningTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
            String filteringTimeMessage = new DecimalFormat("##.###").format(runningTime) + "(sec)";
            messager.submitMessage(LidarImageFusionAPI.DataFilteringTime, filteringTimeMessage);
         }
      };
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
      latestBufferedImage.set(bufferedImage);
   }
}
