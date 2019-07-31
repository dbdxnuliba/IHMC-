package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.awt.image.BufferedImage;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
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

   private final AtomicReference<Integer> bufferSize;

   private final AtomicReference<SegmentationRawDataFilteringParameters> latestSegmentationRawDataFilteringParameters;
   private final AtomicReference<ImageSegmentationParameters> latestImageSegmentationParaeters;
   private final AtomicReference<RawSuperPixelImage> newBuffer = new AtomicReference<>(null);

   public FusedSuperPixelImageBuffer(Messager messager, IntrinsicParameters intrinsic)
   {
      //intrinsicParameters = intrinsic;
      bufferSize = messager.createInput(LidarImageFusionAPI.StereoBufferSize, 50000);
      latestImageSegmentationParaeters = messager.createInput(LidarImageFusionAPI.ImageSegmentationParameters, new ImageSegmentationParameters());
      latestSegmentationRawDataFilteringParameters = messager.createInput(LidarImageFusionAPI.SegmentationRawDataFilteringParameters,
                                                                          new SegmentationRawDataFilteringParameters());

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
      StereoVisionPointCloudMessage pointCloudMessage = latestStereoVisionPointCloudMessage.get();

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

      Point3D[] pointCloud = new Point3D[numberOfPoints];
      int[] colors = new int[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = pointCloudBuffer[i];
         colors[i] = colorBuffer[i];
      }

      fusedSuperPixelImageFactory.setIntrinsicParameters(latestCameraIntrinsicParameters.get());
      fusedSuperPixelImageFactory.setImageSegmentationParameters(latestImageSegmentationParaeters.get());
      fusedSuperPixelImageFactory.setSegmentationRawDataFilteringParameters(latestSegmentationRawDataFilteringParameters.get());
      fusedSuperPixelImageFactory.setCameraPose(latestCameraPosition.get(), latestCameraOrientation.get());

      RawSuperPixelImage data = fusedSuperPixelImageFactory.createLidarImageFusionData(pointCloud, colors, latestBufferedImage.get());

      newBuffer.set(data);
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
