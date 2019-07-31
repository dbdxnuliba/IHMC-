package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.bytedeco.javacpp.indexer.UByteRawIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_ximgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_ximgproc.SuperpixelSLIC;

import boofcv.struct.calib.IntrinsicParameters;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelImage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelData;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;

public class FusedSuperPixelImageFactory
{
   private static final boolean enableDisplaySegmentedContour = true;
   private static final boolean enableDisplayProjectedPointCloud = true;

   private final int bufferedImageType = BufferedImage.TYPE_INT_RGB;
   private final int matType = opencv_core.CV_8UC3;

   private int imageWidth;
   private int imageHeight;

   private BufferedImage segmentedContour;
   private BufferedImage projectedPointCloud;

   private final AtomicReference<IntrinsicParameters> intrinsicParameters = new AtomicReference<>(PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters);
   private final AtomicReference<ImageSegmentationParameters> imageSegmentationParameters = new AtomicReference<>(null);
   private final AtomicReference<SegmentationRawDataFilteringParameters> segmentationRawDataFilteringParameters = new AtomicReference<>(null);
   private final AtomicReference<Point3D> cameraPosition = new AtomicReference<>(new Point3D());
   private final AtomicReference<Quaternion> cameraOrientation = new AtomicReference<>(new Quaternion());

   public RawSuperPixelImage createLidarImageFusionData(Point3D[] pointCloud, int[] colors, BufferedImage bufferedImage)
   {
      imageWidth = bufferedImage.getWidth();
      imageHeight = bufferedImage.getHeight();
      segmentedContour = new BufferedImage(imageWidth, imageHeight, bufferedImageType);
      projectedPointCloud = new BufferedImage(imageWidth, imageHeight, bufferedImageType);

      int[] labels = calculateNewLabelsSLIC(bufferedImage);
      List<RawSuperPixelData> fusionDataSegments = createListOfSegmentationRawData(labels, pointCloud, colors);

      return new RawSuperPixelImage(fusionDataSegments, imageWidth, imageHeight);
   }

   private int[] calculateNewLabelsSLIC(BufferedImage bufferedImage)
   {
      int pixelSize = imageSegmentationParameters.get().getPixelSize();
      double ruler = imageSegmentationParameters.get().getPixelRuler();
      int iterate = imageSegmentationParameters.get().getIterate();
      boolean enableConnectivity = true;
      int elementSize = imageSegmentationParameters.get().getMinElementSize();

      Mat imageMat = convertBufferedImageToMat(bufferedImage);
      Mat convertedMat = new Mat();
      opencv_imgproc.cvtColor(imageMat, convertedMat, opencv_imgproc.COLOR_RGB2HSV);
      SuperpixelSLIC slic = opencv_ximgproc.createSuperpixelSLIC(convertedMat, opencv_ximgproc.SLIC, pixelSize, (float) ruler);
      slic.iterate(iterate);
      if (enableConnectivity)
         slic.enforceLabelConnectivity(elementSize);

      Mat labelMat = new Mat();
      slic.getLabels(labelMat);

      int[] labels = new int[imageWidth * imageHeight];
      for (int i = 0; i < labels.length; i++)
      {
         labels[i] = labelMat.getIntBuffer().get(i);
      }

      return labels;
   }

   private List<RawSuperPixelData> createListOfSegmentationRawData(int[] labelIds, Point3D[] pointCloud, int[] colors)
   {
      if (labelIds.length != imageWidth * imageHeight)
         throw new RuntimeException("newLabels length is different with size of image " + labelIds.length + ", (w)" + imageWidth + ", (h)" + imageHeight);

      List<RawSuperPixelData> rawSuperPixels = new ArrayList<>();

      // create all of the raw super pixels
      int numberOfLabels = max(labelIds) + 1;
      for (int i = 0; i < numberOfLabels; i++)
         rawSuperPixels.add(new RawSuperPixelData(i));

      // projection.
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         if (point == null)
            break;
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, intrinsicParameters.get(), cameraPosition.get(),
                                                                                     cameraOrientation.get());

         if (pixel[0] < 0 || pixel[0] >= imageWidth || pixel[1] < 0 || pixel[1] >= imageHeight)
            continue;

         int arrayIndex = getLabelIdIndex(pixel[0], pixel[1], imageWidth);
         int label = labelIds[arrayIndex];
         rawSuperPixels.get(label).addPoint(new Point3D(point));

         if (enableDisplayProjectedPointCloud)
            projectedPointCloud.setRGB(pixel[0], pixel[1], colors[i]);
      }

      // register adjacent labels.
      for (int u = 1; u < imageWidth - 1; u++)
      {
         for (int v = 1; v < imageHeight - 1; v++)
         {
            registerAdjacentPixelIds(rawSuperPixels, u, v, labelIds, imageWidth);
         }
      }
      // update and calculate normal.
      for (RawSuperPixelData fusionDataSegment : rawSuperPixels)
      {
         if (segmentationRawDataFilteringParameters.get().isEnableFilterFlyingPoint())
            fusionDataSegment.filteringFlyingPoints(segmentationRawDataFilteringParameters.get().getFlyingPointThreshold(),
                                                    segmentationRawDataFilteringParameters.get().getMinimumNumberOfFlyingPointNeighbors());
         fusionDataSegment.update();
         SuperPixelNormalEstimationTools.updateUsingPCA(fusionDataSegment, fusionDataSegment.getPoints());
      }

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

   /**
    * The type of the BufferedImage is TYPE_INT_RGB and the type of the Mat is CV_8UC3.
    */
   private Mat convertBufferedImageToMat(BufferedImage bufferedImage)
   {
      Mat imageMat = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), matType);
      int r, g, b;
      UByteRawIndexer indexer = imageMat.createIndexer();
      for (int y = 0; y < bufferedImage.getHeight(); y++)
      {
         for (int x = 0; x < bufferedImage.getWidth(); x++)
         {
            int rgb = bufferedImage.getRGB(x, y);

            r = (byte) ((rgb >> 0) & 0xFF);
            g = (byte) ((rgb >> 8) & 0xFF);
            b = (byte) ((rgb >> 16) & 0xFF);

            indexer.put(y, x, 0, r);
            indexer.put(y, x, 1, g);
            indexer.put(y, x, 2, b);
         }
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

   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {
      this.intrinsicParameters.set(intrinsicParameters);
   }

   public void setImageSegmentationParameters(ImageSegmentationParameters imageSegmentationParameters)
   {
      this.imageSegmentationParameters.set(imageSegmentationParameters);
   }

   public void setSegmentationRawDataFilteringParameters(SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters)
   {
      this.segmentationRawDataFilteringParameters.set(segmentationRawDataFilteringParameters);
   }

   public void setCameraPose(Point3D position, Quaternion orientation)
   {
      cameraPosition.set(position);
      cameraOrientation.set(orientation);
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
               segmentedSuperPixelsToPack.get(currentLabelId).addAdjacentSegmentLabel(labelOfAdjacentPixel);
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
