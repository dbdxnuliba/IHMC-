package us.ihmc.robotEnvironmentAwareness.fusion.dataFactory;

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
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;

public class SuperPixelGridImageFactory
{
   private static final boolean enableDisplaySegmentedContour = true;
   private static final boolean enableDisplayProjectedPointCloud = false;

   private static final boolean enableConnectivity = true;

   private static final int bufferedImageType = BufferedImage.TYPE_INT_RGB;


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

      int[] labels = calculateNewLabels(stereoImage, imageSegmentationParameters.get());
      List<RawSuperPixelData> rawSuperPixels = populateRawSuperPixelsWithPointCloud(projectedPointCloud, labels, stereoImage, imageHeight, imageWidth,
                                                                                    segmentationRawDataFilteringParameters.get(),
                                                                                    normalEstimationParameters.get());

      return new RawSuperPixelImage(rawSuperPixels, imageWidth, imageHeight);
   }

   private static int[] calculateNewLabels(StereoImage stereoImage, ImageSegmentationParameters imageSegmentationParameters)
   {
      int pixelSize = imageSegmentationParameters.getPixelSize();

      int imageWidth = stereoImage.getWidth();
      int imageHeight = stereoImage.getHeight();
      int numberOfSuperPixelsWide = Math.floorDiv(imageWidth, pixelSize);

      int[] labels = new int[imageWidth * imageHeight];
      for (int u = 0; u < imageWidth; u++)
      {
         int widthNumber = Math.floorDiv(u, pixelSize);
         for (int v = 0; v < imageHeight; v++)
         {
            int heightNumber = Math.floorDiv(v, pixelSize);
            labels[getLabelIdIndex(u, v, imageWidth)] = widthNumber + numberOfSuperPixelsWide * heightNumber;
         }
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
