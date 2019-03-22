package us.ihmc.robotEnvironmentAwareness.slic;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple2D.Point2D;

public class SLICCalculator
{
   private static final int numberOfIteration = 10;

   private static final int nc = 1; // for color distance.
   private static final int ns = 1; // for pixel distance.

   private int[] originalImageBuff;
   private LAPPixel[] imageLABBuff;
   private int[] centerIndices;

   private final int imageSize;
   private final int width;
   private final int height;
   private final int stepSize;

   private final long startTime;

   private void measureComputingTime(String prefix)
   {
      System.out.println(prefix + " " + Conversions.nanosecondsToMilliseconds(System.nanoTime() - startTime) + " (msec)");
   }

   public SLICCalculator(int[] imgBuff, int width, int height, int k)
   {
      startTime = System.nanoTime();

      originalImageBuff = imgBuff;
      System.out.println("width " + width);
      System.out.println("height " + height);

      this.width = width;
      this.height = height;
      imageSize = width * height;
      stepSize = (int) Math.sqrt(width * height / (double) k);
      imageLABBuff = new LAPPixel[imageSize];

      System.out.println("stepSize " + stepSize);

      double[] spaceL = new double[imageSize];
      double[] spaceA = new double[imageSize];
      double[] spaceB = new double[imageSize];
      SLICFunctions.convertRGB2LAB(imgBuff, width, height, spaceL, spaceA, spaceB);
      int pixelIndex = 0;
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {
            imageLABBuff[pixelIndex] = new LAPPixel();
            imageLABBuff[pixelIndex].setLAB(spaceL[pixelIndex], spaceA[pixelIndex], spaceB[pixelIndex]);
            imageLABBuff[pixelIndex].setLocation(j, i);
            pixelIndex++;
         }
      }
      measureComputingTime("initializeTime");
   }

   private void computeInitialCenterIndeces()
   {
      int numberOfGridX = (int) ((double) width / stepSize);
      int remainderPixelsX = width - numberOfGridX * stepSize;

      int numberOfGridY = (int) ((double) height / stepSize);
      int remainderPixelsY = height - numberOfGridY * stepSize;

      System.out.println("numberOfGridX " + numberOfGridX);
      System.out.println("numberOfGridY " + numberOfGridY);
      System.out.println("remainderPixelsX " + remainderPixelsX);
      System.out.println("remainderPixelsY " + remainderPixelsY);

      centerIndices = new int[numberOfGridX * numberOfGridY];
      int k = 0;
      for (int i = 0; i < numberOfGridY; i++)
      {
         for (int j = 0; j < numberOfGridX; j++)
         {
            int centerX = remainderPixelsX / 2 + stepSize / 2 + stepSize * j;
            int centerY = remainderPixelsY / 2 + stepSize / 2 + stepSize * i;

            int centerPixelIndex = centerX + centerY * width;
            centerIndices[k] = centerPixelIndex;
            k++;
         }
      }
   }
   
   private double computeDistance(int centerIndex, LAPPixel pixel)
   {
      int indexOfCenter = centerIndices[centerIndex];
      LAPPixel centerPixel = imageLABBuff[indexOfCenter];
      double dc = Math.pow(centerPixel.getL() - pixel.getL(), 2)
               + Math.pow(centerPixel.getA() - pixel.getA(), 2)
               + Math.pow(centerPixel.getB() - pixel.getB(), 2);

      double ds = Math.pow(centerPixel.getX() - pixel.getX(), 2) + Math.pow(centerPixel.getY() - pixel.getY(), 2);

      return Math.sqrt(dc / nc / nc + ds / ns / ns);
   }

   private int[] getCenterIndicesInSearchingRegion(int x, int y)
   {
      int regionMinX = Math.max(x - stepSize, 0);
      int regionMinY = Math.max(y - stepSize, 0);
      int regionMaxX = Math.min(x + stepSize, width - 1);
      int regionMaxY = Math.min(y + stepSize, height - 1);

      int numberOfCenters = centerIndices.length;
      boolean[] isInRegion = new boolean[numberOfCenters];
      int numberOfCentersInRegion = 0;
      for (int i = 0; i < centerIndices.length; i++)
      {
         int centerY = centerIndices[i] / width;
         int centerX = centerIndices[i] - centerY * width;
         if (centerX >= regionMinX && centerX <= regionMaxX)
         {
            if (centerY >= regionMinY && centerY <= regionMaxY)
            {
               isInRegion[i] = true;
               numberOfCentersInRegion++;
            }
         }
      }

      int[] indicesInRegion = new int[numberOfCentersInRegion];
      int index = 0;
      
      for (int i = 0; i < numberOfCenters; i++)
      {
         if (isInRegion[i])
         {
            indicesInRegion[index] = i;
            index++;
         }
      }

      return indicesInRegion;
   }

   public void compute()
   {
      computeInitialCenterIndeces();

      for (int i = 0; i < 1; i++)
      {
         computeSuperPixel();
         perturbCenters();
      }

      measureComputingTime("compute");
   }

   private void perturbCenters()
   {

   }

   private void computeSuperPixel()
   {
      for (int i = 0; i < imageLABBuff.length; i++)
      {
         LAPPixel pixel = imageLABBuff[i];
         int[] labelCandidates = getCenterIndicesInSearchingRegion(pixel.getX(), pixel.getY());
         if (labelCandidates.length == 0)
            System.out.println("there is no center near this pixel " + i);

         int label = -1;
         double minimumDistance = Double.MAX_VALUE;
         for (int j = 0; j < labelCandidates.length; j++)
         {
            double distanceMeasure = computeDistance(labelCandidates[j], pixel);
            if (distanceMeasure < minimumDistance)
            {
               minimumDistance = distanceMeasure;
               label = labelCandidates[j];
            }
         }
         pixel.setLabel(label);
      }
   }

   public int[] getSegmentedImageBuffer()
   {
      int[] rgbBuff = new int[imageSize];

      for (int i = 0; i < imageSize; i++)
      {
         int label = imageLABBuff[i].getK();
         int centerIndex = centerIndices[label];
         rgbBuff[i] = originalImageBuff[centerIndex];
      }
      return rgbBuff;
   }

   private class LAPPixel
   {
      private double l = 0;
      private double a = 0;
      private double b = 0;

      private int x = 0;
      private int y = 0;

      private int k = -1;

      public void setLAB(double l, double a, double b)
      {
         this.l = l;
         this.a = a;
         this.b = b;
      }

      public void setLocation(int x, int y)
      {
         this.x = x;
         this.y = y;
      }

      public void setLabel(int k)
      {
         this.k = k;
      }

      public double getLABDistance(LAPPixel other)
      {
         return Math.sqrt(Math.pow(l - other.l, 2) + Math.pow(a - other.a, 2) + Math.pow(b - other.b, 2));
      }

      public double getPixelDistance(LAPPixel other)
      {
         return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2));
      }

      public double getL()
      {
         return l;
      }

      public double getA()
      {
         return a;
      }

      public double getB()
      {
         return b;
      }

      public int getX()
      {
         return x;
      }

      public int getY()
      {
         return y;
      }

      public int getK()
      {
         return k;
      }
   }
}
