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
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SegmentedImageRawData;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;

public class LidarImageFusionDataFactory
{
   private static final boolean enableDisplaySegmentedContour = true;
   private static final boolean enableDisplayProjectedPointCloud = true;

   private static final boolean enableConnectivity = true;

   private static final int bufferedImageType = BufferedImage.TYPE_INT_RGB;
   private static final int matType = opencv_core.CV_8UC3;

   private BufferedImage segmentedContour;
   private BufferedImage projectedPointCloud;

   private final AtomicReference<IntrinsicParameters> intrinsicParameters = new AtomicReference<>(PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters);
   private final AtomicReference<ImageSegmentationParameters> imageSegmentationParameters = new AtomicReference<>(null);
   private final AtomicReference<SegmentationRawDataFilteringParameters> segmentationRawDataFilteringParameters = new AtomicReference<>(null);
   private final AtomicReference<Point3D> cameraPosition = new AtomicReference<>(new Point3D());
   private final AtomicReference<Quaternion> cameraOrientation = new AtomicReference<>(new Quaternion());

   public LidarImageFusionData createLidarImageFusionData(Point3D[] pointCloud, int[] colors, BufferedImage bufferedImage)
   {
      int imageWidth = bufferedImage.getWidth();
      int imageHeight = bufferedImage.getHeight();
      segmentedContour = new BufferedImage(imageWidth, imageHeight, bufferedImageType);
      projectedPointCloud = new BufferedImage(imageWidth, imageHeight, bufferedImageType);

      int[] labels = calculateNewLabelsSLIC(bufferedImage, imageWidth, imageHeight, imageSegmentationParameters.get());
      List<SegmentedImageRawData> fusionDataSegments = segmentRawPointCloudIntoSuperpixels(projectedPointCloud, labels, pointCloud, colors, imageHeight, imageWidth, cameraPosition.get(),
                                                                                           cameraOrientation.get(), intrinsicParameters.get(),
                                                                                           segmentationRawDataFilteringParameters.get());

      return new LidarImageFusionData(fusionDataSegments, imageWidth, imageHeight);
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

   private static int[] calculateNewLabelsSLIC(BufferedImage bufferedImage, int imageWidth, int imageHeight, ImageSegmentationParameters imageSegmentationParameters)
   {
      int pixelSize = imageSegmentationParameters.getPixelSize();
      double ruler = imageSegmentationParameters.getPixelRuler();
      int iterate = imageSegmentationParameters.getIterate();
      int elementSize = imageSegmentationParameters.getMinElementSize();

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

   private static List<SegmentedImageRawData> segmentRawPointCloudIntoSuperpixels(BufferedImage projectedPointCloudToPack, int[] labels, Point3D[] pointCloud, int[] colors, int imageHeight, int imageWidth,
                                                                           Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation,
                                                                           IntrinsicParameters intrinsicParameters,
                                                                           SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters)
   {
      if (labels.length != imageWidth * imageHeight)
         throw new RuntimeException("newLabels length is different with size of image " + labels.length + ", (w)" + imageWidth + ", (h)" + imageHeight);

      List<SegmentedImageRawData> fusionDataSegments = new ArrayList<SegmentedImageRawData>();

      // create.
      TIntArrayList labelList = new TIntArrayList(labels);
      int numberOfLabels = labelList.max() + 1;
      for (int i = 0; i < numberOfLabels; i++)
         fusionDataSegments.add(new SegmentedImageRawData(i));

      // projection.
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3DReadOnly pointToProject = pointCloud[i];
         if (pointToProject == null)
            break;
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(pointToProject, intrinsicParameters, cameraPosition,
                                                                                     cameraOrientation);

         if (pixel[0] < 0 || pixel[0] >= imageWidth || pixel[1] < 0 || pixel[1] >= imageHeight)
            continue;

         int arrayIndex = getArrayIndex(pixel[0], pixel[1], imageWidth);
         int label = labels[arrayIndex];
         // TODO does this need to be a copied point?
         fusionDataSegments.get(label).addPoint(new Point3D(pointToProject));

         if (enableDisplayProjectedPointCloud)
            projectedPointCloudToPack.setRGB(pixel[0], pixel[1], colors[i]);
      }

      // register adjacent labels.
      for (int u = 1; u < imageWidth - 1; u++)
      {
         for (int v = 1; v < imageHeight - 1; v++)
         {
            int curLabel = labels[getArrayIndex(u, v, imageWidth)];
            int[] labelsOfAdjacentPixels = new int[4];
            labelsOfAdjacentPixels[0] = labels[getArrayIndex(u, v - 1, imageWidth)]; // N
            labelsOfAdjacentPixels[1] = labels[getArrayIndex(u, v + 1, imageWidth)]; // S
            labelsOfAdjacentPixels[2] = labels[getArrayIndex(u - 1, v, imageWidth)]; // W
            labelsOfAdjacentPixels[3] = labels[getArrayIndex(u + 1, v, imageWidth)]; // E

            for (int labelOfAdjacentPixel : labelsOfAdjacentPixels)
            {
               if (curLabel != labelOfAdjacentPixel)
               {
                  if (!fusionDataSegments.get(curLabel).contains(labelOfAdjacentPixel))
                     fusionDataSegments.get(curLabel).addAdjacentSegmentLabel(labelOfAdjacentPixel);
               }
            }
         }
      }

      // update and calculate normal.
      for (SegmentedImageRawData fusionDataSegment : fusionDataSegments)
      {
         if (segmentationRawDataFilteringParameters.isEnableFilterFlyingPoint())
            fusionDataSegment.filterOutFlyingPoints(segmentationRawDataFilteringParameters.getFlyingPointThreshold(),
                                                    segmentationRawDataFilteringParameters.getMinimumNumberOfFlyingPointNeighbors());
         fusionDataSegment.update();
      }

      // set segment center in 2D.
      int[] totalU = new int[numberOfLabels];
      int[] totalV = new int[numberOfLabels];
      int[] numberOfPixels = new int[numberOfLabels];

      for (int widthIndex = 0; widthIndex < imageWidth; widthIndex++)
      {
         for (int heightIndex = 0; heightIndex < imageHeight; heightIndex++)
         {
            int label = labels[getArrayIndex(widthIndex, heightIndex, imageWidth)];
            totalU[label] += widthIndex;
            totalV[label] += heightIndex;
            numberOfPixels[label]++;
         }
      }

      for (int i = 0; i < numberOfLabels; i++)
      {
         fusionDataSegments.get(i).setSegmentCenter(totalU[i] / numberOfPixels[i], totalV[i] / numberOfPixels[i]);
      }

      return fusionDataSegments;
   }

   /**
    * The type of the BufferedImage is TYPE_INT_RGB and the type of the Mat is CV_8UC3.
    */
   private static Mat convertBufferedImageToMat(BufferedImage bufferedImage)
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

   private static int getArrayIndex(int u, int v, int imageWidth)
   {
      return u + v * imageWidth;
   }


}
