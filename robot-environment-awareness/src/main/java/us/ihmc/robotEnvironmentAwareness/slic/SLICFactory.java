package us.ihmc.robotEnvironmentAwareness.slic;

import us.ihmc.euclid.tuple2D.Point2D;

public class SLICFactory
{
   private final int numberOfIteration = 10;

   private final int stepSize;
   
   private final int nc = 1; // color distance.
   private final int ns = 1; // pixel distance.

   private final double[] spaceL;
   private final double[] spaceA;
   private final double[] spaceB;

   private double[][] spacesOfCenter;

   public SLICFactory(int[] imgBuff, int width, int height, int k)
   {
      int imageSize = width * height;
      stepSize = (int) Math.sqrt(width * height / (double) k);

      spaceL = new double[imageSize];
      spaceA = new double[imageSize];
      spaceB = new double[imageSize];

      convertRGB2LAB(imgBuff, width, height, spaceL, spaceA, spaceB);

      spacesOfCenter = new double[k][5];
      for (int i = 0; i < k; i++)
         spacesOfCenter[i] = new double[5];
   }

   private double computeDistance(int clusterIndex, double l, double a, double b, double x, double y)
   {
      double dc = Math.pow(spacesOfCenter[clusterIndex][0] - l, 2) + Math.pow(spacesOfCenter[clusterIndex][1] - a, 2)
            + Math.pow(spacesOfCenter[clusterIndex][2] - b, 2);

      double ds = Math.pow(spacesOfCenter[clusterIndex][0] - x, 2) + Math.pow(spacesOfCenter[clusterIndex][1] - y, 2);

      return Math.sqrt(dc / nc / nc + ds / ns / ns);
   }
   
   private Point2D findLocalMinimum()
   {
      Point2D localMinimum = new Point2D();
      
      return null;
   }

   private void generateSuperPixels()
   {

   }

   private void createConnectivity()
   {

   }

   private void drawContours()
   {

   }

   private void drawCenterGrid()
   {

   }

   private void colorWithClusterMeans()
   {

   }

   public void compute()
   {
      generateSuperPixels();
      createConnectivity();
   }

   public int[] getSegmentedImageBuffer()
   {
      return null;
   }
   
   private class LAPPixel
   {
      private double l;
      private double a;
      private double b;
      
      private int x;
      private int y;
      
      public LAPPixel(double l, double a, double b, int x, int y)
      {
         this.l = l;
         this.a = a;
         this.b = b;
         
         this.x = x;
         this.y = y;
      }
      
      public double getLABDistance(LAPPixel other)
      {
         return 0.0;
      }
   }

   private static final double epsilonOfCIEStandard = 0.008856;
   private static final double kappaOfCIEStandard = 903.3;

   private static void convertStandardRGB2XYZ(int sR, int sG, int sB, double[] xyzToPack)
   {
      double standardRValue = sR / 255.0;
      double standardGValue = sG / 255.0;
      double standardBValue = sB / 255.0;

      double r, g, b;

      if (standardRValue <= 0.04045)
         r = standardRValue / 12.92;
      else
         r = Math.pow((standardRValue + 0.055 / 1.055), 2.4);

      if (standardGValue <= 0.04045)
         g = standardGValue / 12.92;
      else
         g = Math.pow((standardGValue + 0.055 / 1.055), 2.4);

      if (standardBValue <= 0.04045)
         b = standardBValue / 12.92;
      else
         b = Math.pow((standardBValue + 0.055 / 1.055), 2.4);

      xyzToPack[0] = r * 0.4124564 + g * 0.3575761 + b * 0.1804375;
      xyzToPack[1] = r * 0.2126729 + g * 0.7151522 + b * 0.0721750;
      xyzToPack[2] = r * 0.0193339 + g * 0.1191920 + b * 0.9503041;
   }

   /**
    * under Illuminant D65. equation is from following link.
    * https://en.wikipedia.org/wiki/CIELAB_color_space
    */
   private static void convertStandardRGB2LAB(int sR, int sG, int sB, double[] labToPack)
   {
      double[] xyz = new double[3];
      convertStandardRGB2XYZ(sR, sG, sB, xyz);

      double referenceX = 0.95047;
      double referenceY = 1.0;
      double referenceZ = 1.08883;

      double normalizedX = xyz[0] / referenceX;
      double normalizedY = xyz[1] / referenceY;
      double normalizedZ = xyz[2] / referenceZ;

      double fx = 0, fy = 0, fz = 0;

      if (normalizedX > epsilonOfCIEStandard)
         fx = Math.pow(normalizedX, 1.0 / 3.0);
      else
         fx = (kappaOfCIEStandard * normalizedX + 16.0) / 116.0;

      if (normalizedY > epsilonOfCIEStandard)
         fy = Math.pow(normalizedY, 1.0 / 3.0);
      else
         fy = (kappaOfCIEStandard * normalizedY + 16.0) / 116.0;

      if (normalizedZ > epsilonOfCIEStandard)
         fz = Math.pow(normalizedZ, 1.0 / 3.0);
      else
         fz = (kappaOfCIEStandard * normalizedZ + 16.0) / 116.0;

      labToPack[0] = 116.0 * fy - 16.0;
      labToPack[1] = 500.0 * (fx - fy);
      labToPack[2] = 200.0 * (fy - fz);
   }

   private static void convertRGB2LAB(int[] imgBuffer, int width, int height, double[] lBufferToPack, double[] aBufferToPack, double[] bBufferToPack)
   {
      int size = width * height;
      double[] labVal = new double[3];

      lBufferToPack = new double[size];
      aBufferToPack = new double[size];
      bBufferToPack = new double[size];

      int r, g, b;
      for (int j = 0; j < size; j++)
      {
         r = (imgBuffer[j] >> 16) & 0xFF;
         g = (imgBuffer[j] >> 8) & 0xFF;
         b = (imgBuffer[j]) & 0xFF;

         convertStandardRGB2LAB(r, g, b, labVal);

         lBufferToPack[j] = labVal[0];
         aBufferToPack[j] = labVal[1];
         bBufferToPack[j] = labVal[2];
      }
   }
}
