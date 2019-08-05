package us.ihmc.robotEnvironmentAwareness.fusion.dataFactory;

import org.bytedeco.javacpp.indexer.UByteRawIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_ximgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_ximgproc.SuperpixelSLIC;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelImage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoImage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoPoint;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.StereoREAParallelParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SegmentationRawDataFiltering;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.SuperPixelNormalEstimationTools;

import java.awt.image.BufferedImage;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;

public class RawSuperPixelImageFactory
{
   private static final boolean enableDisplaySegmentedContour = true;
   private static final boolean enableDisplayProjectedPointCloud = false;

   private static final boolean enableConnectivity = true;

   private static final int bufferedImageType = BufferedImage.TYPE_INT_RGB;
   private static final int matType = opencv_core.CV_8UC3;


   private BufferedImage segmentedContour;
   private BufferedImage projectedPointCloud;

   private final AtomicReference<ImageSegmentationParameters> imageSegmentationParameters = new AtomicReference<>(null);
   private final AtomicReference<SegmentationRawDataFilteringParameters> segmentationRawDataFilteringParameters = new AtomicReference<>(null);
   private final AtomicReference<SuperPixelNormalEstimationParameters> normalEstimationParameters = new AtomicReference<>(null);

   public RawSuperPixelImage createRawSuperPixelImage(StereoImage stereoImage)
   {
      int imageWidth = stereoImage.getWidth();
      int imageHeight = stereoImage.getHeight();
      segmentedContour = new BufferedImage(imageWidth, imageHeight, bufferedImageType);
      projectedPointCloud = new BufferedImage(imageWidth, imageHeight, bufferedImageType);

      int[] labels = calculateNewLabelsSLIC(stereoImage, imageSegmentationParameters.get());
      List<RawSuperPixelData> rawSuperPixels = populateRawSuperPixelsWithPointCloud(projectedPointCloud, labels, stereoImage, imageHeight, imageWidth,
                                                                                    segmentationRawDataFilteringParameters.get(),
                                                                                    normalEstimationParameters.get());

      return new RawSuperPixelImage(rawSuperPixels, imageWidth, imageHeight);
   }

   private static int[] calculateNewLabelsSLIC(StereoImage stereoImage, ImageSegmentationParameters imageSegmentationParameters)
   {
      int pixelSize = imageSegmentationParameters.getPixelSize();
      double ruler = imageSegmentationParameters.getPixelRuler();
      int iterate = imageSegmentationParameters.getIterate();
      int elementSize = imageSegmentationParameters.getMinElementSize();

      Mat imageMat = convertStereoImageToMat(stereoImage);
      Mat convertedMat = new Mat();
      opencv_imgproc.cvtColor(imageMat, convertedMat, opencv_imgproc.COLOR_RGB2HSV);
      SuperpixelSLIC slic = opencv_ximgproc.createSuperpixelSLIC(convertedMat, opencv_ximgproc.SLIC, pixelSize, (float) ruler);
      slic.iterate(iterate);
      if (imageSegmentationParameters.getEnableConnectivity())
         slic.enforceLabelConnectivity(elementSize);

      Mat labelMat = new Mat();
      slic.getLabels(labelMat);

      int[] labels = new int[stereoImage.getWidth() * stereoImage.getHeight()];
      IntBuffer intBuffer = labelMat.getIntBuffer();
      for (int i = 0; i < labels.length; i++)
      {
         labels[i] = intBuffer.get(i);
      }

      return labels;
   }


   private static List<RawSuperPixelData> populateRawSuperPixelsWithPointCloud(BufferedImage projectedPointCloudToPack, int[] labelIds,
                                                                               StereoImage stereoImage, int imageHeight, int imageWidth,
                                                                               SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters,
                                                                               SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      if (labelIds.length != imageWidth * imageHeight)
         throw new RuntimeException("newLabels length is different with size of image " + labelIds.length + ", (w)" + imageWidth + ", (h)" + imageHeight);

      List<RawSuperPixelData> rawSuperPixels = new ArrayList<>();

      // create all of the raw super pixels
      int numberOfLabels = max(labelIds) + 1;
      for (int i = 0; i < numberOfLabels; i++)
         rawSuperPixels.add(new RawSuperPixelData(i));

      // projection.
      for (StereoPoint stereoPoint : stereoImage.getPoints())
      {
         projectColoredPixelIntoSuperPixel(projectedPointCloudToPack, rawSuperPixels, labelIds, stereoPoint, imageWidth);
      }

      // register adjacent labels.
      for (int widthIndex = 1; widthIndex < imageWidth - 1; widthIndex++)
      {
         for (int heightIndex = 1; heightIndex < imageHeight - 1; heightIndex++)
         {
            registerAdjacentPixelIds(rawSuperPixels, widthIndex, heightIndex, labelIds, imageWidth);
         }
      }

      // updateNewBuffer and updateNewBuffer normal.
      Stream<RawSuperPixelData> superPixelStream = StereoREAParallelParameters.updateRawSuperPixelNormalsInParallel ? rawSuperPixels.parallelStream() : rawSuperPixels.stream();
      superPixelStream.forEach(superPixel -> updateSuperpixelAndCalculateNormal(superPixel, segmentationRawDataFilteringParameters, normalEstimationParameters));

      // set segment center in 2D.
      int[] totalU = new int[numberOfLabels];
      int[] totalV = new int[numberOfLabels];
      int[] numberOfPixels = new int[numberOfLabels];

      for (int widthIndex = 0; widthIndex < imageWidth; widthIndex++)
      {
         for (int heightIndex = 0; heightIndex < imageHeight; heightIndex++)
         {
            int labelId = labelIds[getLabelIdIndex(widthIndex, heightIndex, imageWidth)];
            totalU[labelId] += widthIndex;
            totalV[labelId] += heightIndex;
            numberOfPixels[labelId]++;
         }
      }

      for (int i = 0; i < numberOfLabels; i++)
      {
         rawSuperPixels.get(i).setSegmentCenter(totalU[i] / numberOfPixels[i], totalV[i] / numberOfPixels[i]);
      }

      return rawSuperPixels;
   }

   private static void projectColoredPixelIntoSuperPixel(BufferedImage projectedPointCloudToPack, List<RawSuperPixelData> segmentedSuperPixelToPack,
                                                         int[] labelIds, StereoPoint stereoPoint, int imageWidth)
   {
      if (stereoPoint == null)
         return;

      int[] pixelIndices = stereoPoint.getPixelIndices();
      int labelId = labelIds[getLabelIdIndex(pixelIndices[0], pixelIndices[1], imageWidth)];
      segmentedSuperPixelToPack.get(labelId).addPoint(new Point3D(stereoPoint));

      if (enableDisplayProjectedPointCloud)
         projectedPointCloudToPack.setRGB(pixelIndices[0], pixelIndices[1], stereoPoint.getColor());
   }

   private static void updateSuperpixelAndCalculateNormal(RawSuperPixelData rawSuperPixel,
                                                          SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters,
                                                          SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      SegmentationRawDataFiltering.filterOutFlyingPoints(rawSuperPixel, segmentationRawDataFilteringParameters);

      rawSuperPixel.updateAdjacency();

      if (normalEstimationParameters.updateUsingPCA())
         SuperPixelNormalEstimationTools.updateUsingPCA(rawSuperPixel, rawSuperPixel.getPointsInPixel(), StereoREAParallelParameters.addPointsToRawPCAInParallel);
      else
         SuperPixelNormalEstimationTools.updateUsingRansac(rawSuperPixel, rawSuperPixel.getPointsInPixel(), normalEstimationParameters);
   }

   /**
    * The type of the BufferedImage is TYPE_INT_RGB and the type of the Mat is CV_8UC3.
    */
   private static Mat convertStereoImageToMat(StereoImage bufferedImage)
   {
      Mat imageMat = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), matType);
      UByteRawIndexer indexer = imageMat.createIndexer();
      for (StereoPoint point : bufferedImage.getPoints())
      {
         int rgb = point.getColor();
         int x = point.getXIndex();
         int y = point.getYIndex();

         indexer.put(y, x, 0, (byte) ((rgb /*>> 0*/) & 0xFF));
         indexer.put(y, x, 1, (byte) ((rgb >> 8) & 0xFF));
         indexer.put(y, x, 2, (byte) ((rgb >> 16) & 0xFF));
      }

      indexer.release();

      return imageMat;
   }


   public BufferedImage getSegmentedContourBufferedImage()
   {
      return segmentedContour;
   }

   public BufferedImage getProjectedPointCloudBufferedImage()
   {
      return projectedPointCloud;
   }

   public void setImageSegmentationParameters(ImageSegmentationParameters imageSegmentationParameters)
   {
      this.imageSegmentationParameters.set(imageSegmentationParameters);
   }

   public void setSegmentationRawDataFilteringParameters(SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters)
   {
      this.segmentationRawDataFilteringParameters.set(segmentationRawDataFilteringParameters);
   }

   public void setNormalEstimationParameters(SuperPixelNormalEstimationParameters normalEstimationParameters)
   {
      this.normalEstimationParameters.set(normalEstimationParameters);
   }

   private static void registerAdjacentPixelIds(List<RawSuperPixelData> segmentedSuperPixelsToPack, int widthIndex, int heightIndex, int[] labelIds, int imageWidth)
   {
      int currentLabelId = labelIds[getLabelIdIndex(widthIndex, heightIndex, imageWidth)];
      int[] idOfAdjacentPixels = new int[4];
      idOfAdjacentPixels[0] = labelIds[getLabelIdIndex(widthIndex, heightIndex - 1, imageWidth)]; // N
      idOfAdjacentPixels[1] = labelIds[getLabelIdIndex(widthIndex, heightIndex + 1, imageWidth)]; // S
      idOfAdjacentPixels[2] = labelIds[getLabelIdIndex(widthIndex - 1, heightIndex, imageWidth)]; // W
      idOfAdjacentPixels[3] = labelIds[getLabelIdIndex(widthIndex + 1, heightIndex, imageWidth)]; // E

      for (int labelOfAdjacentPixel : idOfAdjacentPixels)
      {
         if (currentLabelId != labelOfAdjacentPixel)
         {
            if (!segmentedSuperPixelsToPack.get(currentLabelId).contains(labelOfAdjacentPixel))
               segmentedSuperPixelsToPack.get(currentLabelId).addAdjacentPixel(labelOfAdjacentPixel);
         }
      }
   }

   private static int getLabelIdIndex(int u, int v, int imageWidth)
   {
      return u + v * imageWidth;
   }

   private static int max(int[] values)
   {
      int max = Integer.MIN_VALUE;
      for (int i = 0; i < values.length; i++)
      {
         if (values[i] > max)
         {
            max = values[i];
         }
      }
      return max;
   }

}
