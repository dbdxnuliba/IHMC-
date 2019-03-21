package us.ihmc.robotEnvironmentAwareness.slic;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.log.LogTools;

public final class SLICFunctions
{
   private static final double epsilonOfCIEStandard = 0.008856;
   private static final double kappaOfCIEStandard = 903.3;

   private static int[] dx4 = {-1, 0, 1, 0};
   private static int[] dy4 = {0, -1, 0, 1};

   private static double[] m_lvec;
   private static double[] m_avec;
   private static double[] m_bvec;

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

   private static void detectLABEdges(double[] lvec, double[] avec, double[] bvec, int width, int height, double[] edges)
   {
      for (int j = 1; j < height - 1; j++)
      {
         for (int k = 1; k < width - 1; k++)
         {
            int i = j * width + k;
            double dx = (lvec[i - 1] - lvec[i + 1]) * (lvec[i - 1] - lvec[i + 1]) + (avec[i - 1] - avec[i + 1]) * (avec[i - 1] - avec[i + 1])
                  + (bvec[i - 1] - bvec[i + 1]) * (bvec[i - 1] - bvec[i + 1]);

            double dy = (lvec[i - width] - lvec[i + width]) * (lvec[i - width] - lvec[i + width])
                  + (avec[i - width] - avec[i + width]) * (avec[i - width] - avec[i + width])
                  + (bvec[i - width] - bvec[i + width]) * (bvec[i - width] - bvec[i + width]);

            // TODO : compare the difference.
            //edges[i] = (dx * dx + dy * dy);
            edges[i] = (dx + dy);
         }
      }
   }

   private static void perturbSeeds(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, int width, int height, double[] edges)
   {
      int[] dx8 = {-1, -1, 0, 1, 1, 1, 0, -1};
      int[] dy8 = {0, -1, -1, -1, 0, 1, 1, 1};

      int numseeds = seedsL.length;

      for (int n = 0; n < numseeds; n++)
      {
         int ox = (int) seedsX[n]; //original x
         int oy = (int) seedsY[n]; //original y
         int oind = oy * height + ox;

         int storeind = oind;
         for (int i = 0; i < 8; i++)
         {
            int nx = ox + dx8[i]; //new x
            int ny = oy + dy8[i]; //new y

            if (nx >= 0 && nx < width && ny >= 0 && ny < height)
            {
               int nind = ny * width + nx;
               if (edges[nind] < edges[storeind])
                  storeind = nind;
            }
         }
         if (storeind != oind)
         {
            seedsX[n] = storeind % width;
            seedsY[n] = storeind / width;
            seedsL[n] = m_lvec[storeind];
            seedsA[n] = m_avec[storeind];
            seedsB[n] = m_bvec[storeind];
         }
      }
   }

   private static void getLABXYSeedsForGivenStepSize(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, int width, int height,
                                                     int stepSize, boolean perturbseeds, double[] edgeMagnitude)
   {
      int n = 0;

      int xstrips = (int) ((0.5 + (double) width) / ((double) stepSize));
      int ystrips = (int) ((0.5 + (double) height) / ((double) stepSize));

      int xerr = width - stepSize * xstrips;
      int yerr = height - stepSize * ystrips;

      double xerrperstrip = (double) xerr / (double) xstrips;
      double yerrperstrip = (double) yerr / (double) ystrips;

      int xoff = stepSize / 2;
      int yoff = stepSize / 2;

      for (int y = 0; y < ystrips; y++)
      {
         int ye = (int) (y * yerrperstrip);
         for (int x = 0; x < xstrips; x++)
         {
            int xe = (int) (x * xerrperstrip);
            int i = (y * stepSize + yoff + ye) * width + (x * stepSize + xoff + xe);

            seedsL[n] = m_lvec[i];
            seedsA[n] = m_avec[i];
            seedsB[n] = m_bvec[i];
            seedsX[n] = (x * stepSize + xoff + xe);
            seedsY[n] = (y * stepSize + yoff + ye);
            n++;
         }
      }

      if (perturbseeds)
         perturbSeeds(seedsL, seedsA, seedsB, seedsX, seedsY, width, height, edgeMagnitude);
   }
   //
   //   public static void performSuperpixelSLIC(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, int width, int height,
   //                                            int[] kLabels, int stepSize, double[] edgeMagnitude, double m)
   //   {
   //      int size = width * height;
   //      int numk = seedsL.length;
   //      int offset = stepSize;
   //
   //      double[] clustersize = new double[numk];
   //      double[] inv = new double[numk]; //to store 1/clustersize[k] values
   //
   //      double[] sigmal = new double[numk];
   //      double[] sigmaa = new double[numk];
   //      double[] sigmab = new double[numk];
   //      double[] sigmax = new double[numk];
   //      double[] sigmay = new double[numk];
   //      double[] distvec = new double[size];
   //
   //      for (int i_ = 0; i_ < numk; i_++)
   //      {
   //         clustersize[i_] = 0;
   //         inv[i_] = 0;
   //         sigmal[i_] = 0;
   //         sigmaa[i_] = 0;
   //         sigmab[i_] = 0;
   //         sigmax[i_] = 0;
   //         sigmay[i_] = 0;
   //
   //      }
   //      for (int i_ = 0; i_ < distvec.length; i_++)
   //      {
   //         distvec[i_] = Double.MAX_VALUE;
   //      }
   //
   //      double invwt = 1.0 / ((stepSize / m) * (m));
   //
   //      int x1, y1, x2, y2;
   //      double l, a, b;
   //      double dist;
   //      double distxy;
   //      for (int itr = 0; itr < 10; itr++)
   //      {
   //         for (int i_ = 0; i_ < distvec.length; i_++)
   //         {
   //            distvec[i_] = Double.MAX_VALUE;
   //         }
   //         for (int n = 0; n < numk; n++)
   //         {
   //            y1 = (int) Math.max(0, seedsY[n] - offset);
   //            y2 = (int) Math.min(height, seedsY[n] + offset);
   //            x1 = (int) Math.max(0, seedsX[n] - offset);
   //            x2 = (int) Math.min(width, seedsX[n] + offset);
   //
   //            for (int y = y1; y < y2; y++)
   //            {
   //               for (int x = x1; x < x2; x++)
   //               {
   //                  int i = y * width + x;
   //
   //                  l = m_lvec[i];
   //                  a = m_avec[i];
   //                  b = m_bvec[i];
   //
   //                  dist = (l - seedsL[n]) * (l - seedsL[n]) + (a - seedsA[n]) * (a - seedsA[n]) + (b - seedsB[n]) * (b - seedsB[n]);
   //
   //                  distxy = (x - seedsX[n]) * (x - seedsX[n]) + (y - seedsY[n]) * (y - seedsY[n]);
   //
   //                  //------------------------------------------------------------------------
   //                  dist += distxy * invwt;//dist = sqrt(dist) + sqrt(distxy*invwt);//this is more exact
   //                  //------------------------------------------------------------------------
   //
   //                  if (dist < distvec[i])
   //                  {
   //                     distvec[i] = dist;
   //                     kLabels[i] = n;
   //                  }
   //               }
   //            }
   //         }
   //         //-----------------------------------------------------------------
   //         // Recalculate the centroid and store in the seed values
   //         //-----------------------------------------------------------------
   //         //instead of reassigning memory on each iteration, just reset.
   //
   //         for (int i_ = 0; i_ < numk; i_++)
   //         {
   //            clustersize[i_] = 0;
   //            sigmal[i_] = 0;
   //            sigmaa[i_] = 0;
   //            sigmab[i_] = 0;
   //            sigmax[i_] = 0;
   //            sigmay[i_] = 0;
   //         }
   //
   //         //------------------------------------
   //         //edgesum.assign(numk, 0);
   //         //------------------------------------
   //
   //         int ind = 0;
   //         for (int r = 0; r < height; r++)
   //         {
   //            for (int c = 0; c < width; c++)
   //            {
   //               sigmal[kLabels[ind]] += m_lvec[ind];
   //               sigmaa[kLabels[ind]] += m_avec[ind];
   //               sigmab[kLabels[ind]] += m_bvec[ind];
   //               sigmax[kLabels[ind]] += c;
   //               sigmay[kLabels[ind]] += r;
   //               //------------------------------------
   //               //edgesum[klabels[ind]] += edgemag[ind];
   //               //------------------------------------
   //               clustersize[kLabels[ind]] += 1.0;
   //               ind++;
   //            }
   //         }
   //
   //         for (int k = 0; k < numk; k++)
   //         {
   //            if (clustersize[k] <= 0)
   //            {
   //               clustersize[k] = 1;
   //            }
   //            inv[k] = 1.0 / clustersize[k];//computing inverse now to multiply, than divide later
   //         }
   //
   //         for (int k = 0; k < numk; k++)
   //         {
   //            seedsL[k] = sigmal[k] * inv[k];
   //            seedsA[k] = sigmaa[k] * inv[k];
   //            seedsB[k] = sigmab[k] * inv[k];
   //            seedsX[k] = sigmax[k] * inv[k];
   //            seedsY[k] = sigmay[k] * inv[k];
   //            //------------------------------------
   //            //edgesum[k] *= inv[k];
   //            //------------------------------------
   //         }
   //      }
   //   }

   private static void getLABXYSeedsForGivenK(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, int width, int height, int k,
                                              boolean perturbSeeds, double[] edgeMagnitude)
   {
      int sz = width * height;
      double step = Math.sqrt((double) (sz) / (double) (k));
      int xoff = (int) step / 2;
      int yoff = (int) step / 2;

      int n = 0;
      int r = 0;
      for (int y = 0; y < height; y++)
      {
         int Y = (int) (y * step + yoff);
         if (Y > height - 1)
            break;

         for (int x = 0; x < width; x++)
         {
            int X = (int) (x * step + (xoff << (r & 0x1))); //hex grid
            if (X > width - 1)
               break;

            int i = Y * width + X;

            if (n < k)
            {
               seedsL[i] = (m_lvec[i]);
               seedsA[i] = (m_avec[i]);
               seedsB[i] = (m_bvec[i]);
               seedsX[i] = (X);
               seedsY[i] = (Y);
               n++;
            }
            else
            {
               LogTools.info("n is over the given K " + n);
               continue;
            }
         }
         r++;
      }

      if (perturbSeeds)
         perturbSeeds(seedsL, seedsA, seedsB, seedsX, seedsY, width, height, edgeMagnitude);
   }

   private static void performSuperPixelSegmentationVariableSAndM(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY,
                                                                  int width, int height, int[] kLabels, int stepSize, int numberOfIteration)
   {
      int size = width * height;
      int numk = seedsL.length;
      int numitr = 0;

      int offset = stepSize;
      if (stepSize < 10)
         offset = (int) (stepSize * 1.5);

      double[] sigmal = new double[numk];
      double[] sigmaa = new double[numk];
      double[] sigmab = new double[numk];
      double[] sigmax = new double[numk];
      double[] sigmay = new double[numk];

      int[] clustersize = new int[numk];
      double[] inv = new double[numk];
      double[] distxy = new double[size];
      double[] distlab = new double[size];
      double[] distvec = new double[size];
      for (int i = 0; i < distxy.length; i++)
      {
         distxy[i] = Double.MAX_VALUE;
         distlab[i] = Double.MAX_VALUE;
         distvec[i] = Double.MAX_VALUE;
      }

      double[] maxlab = new double[numk];
      double[] maxxy = new double[numk];
      for (int i = 0; i < maxlab.length; i++)
      {
         maxlab[i] = 10.0 * 10.0;
         maxxy[i] = stepSize * stepSize;
      }

      double invxywt = 1.0 / (stepSize * stepSize);//NOTE: this is different from how usual SLIC/LKM works

      while (numitr < numberOfIteration)
      {
         numitr++;
         for (int i = 0; i < size; i++)
            distvec[i] = Double.MAX_VALUE;

         for (int n = 0; n < numk; n++)
         {
            int y1 = (int) Math.max(0.0, seedsY[n] - offset);
            int y2 = (int) Math.min((double) height, seedsY[n] + offset);
            int x1 = (int) Math.max(0.0, seedsX[n] - offset);
            int x2 = (int) Math.min((double) width, seedsX[n] + offset);

            for (int y = y1; y < y2; y++)
            {
               for (int x = x1; x < x2; x++)
               {
                  int i = y * width + x;
                  assert (y < height && x < width && y >= 0 && x >= 0);

                  double l = m_lvec[i];
                  double a = m_avec[i];
                  double b = m_bvec[i];

                  distlab[i] = (l - seedsL[n]) * (l - seedsL[n]) + (a - seedsA[n]) * (a - seedsA[n]) + (b - seedsB[n]) * (b - seedsB[n]);
                  distxy[i] = (x - seedsX[n]) * (x - seedsX[n]) + (y - seedsY[n]) * (y - seedsY[n]);

                  //------------------------------------------------------------------------
                  double dist = distlab[i] / maxlab[n] + distxy[i] * invxywt;//only varying m, prettier superpixels
                  //double dist = distlab[i]/maxlab[n] + distxy[i]/maxxy[n];//varying both m and S
                  //------------------------------------------------------------------------

                  if (dist < distvec[i])
                  {
                     distvec[i] = dist;
                     kLabels[i] = n;
                  }
               }
            }
         }
         //-----------------------------------------------------------------
         // Assign the max color distance for a cluster
         //-----------------------------------------------------------------
         if (0 == numitr)
         {
            for (int i = 0; i < numk; i++)
            {
               maxlab[i] = 1.0;
               maxxy[i] = 1.0;
            }
         }
         {
            for (int i = 0; i < size; i++)
            {
               if (maxlab[kLabels[i]] < distlab[i])
                  maxlab[kLabels[i]] = distlab[i];
               if (maxxy[kLabels[i]] < distxy[i])
                  maxxy[kLabels[i]] = distxy[i];
            }
         }
         //-----------------------------------------------------------------
         // Recalculate the centroid and store in the seed values
         //-----------------------------------------------------------------
         for (int i = 0; i < numk; i++)
         {
            sigmal[i] = 0.0;
            sigmaa[i] = 0.0;
            sigmab[i] = 0.0;
            sigmax[i] = 0.0;
            sigmay[i] = 0.0;

            clustersize[i] = 0;
         }

         for (int j = 0; j < size; j++)
         {
            int temp = kLabels[j];
            assert (kLabels[j] >= 0);
            sigmal[kLabels[j]] += m_lvec[j];
            sigmaa[kLabels[j]] += m_avec[j];
            sigmab[kLabels[j]] += m_bvec[j];
            sigmax[kLabels[j]] += (j % width);
            sigmay[kLabels[j]] += (j / width);

            clustersize[kLabels[j]]++;
         }

         {
            for (int k = 0; k < numk; k++)
            {
               if (clustersize[k] <= 0)
                  clustersize[k] = 1;
               inv[k] = 1.0 / (double) (clustersize[k]);//computing inverse now to multiply, than divide later
            }
         }

         {
            for (int k = 0; k < numk; k++)
            {
               seedsL[k] = sigmal[k] * inv[k];
               seedsA[k] = sigmaa[k] * inv[k];
               seedsB[k] = sigmab[k] * inv[k];
               seedsX[k] = sigmax[k] * inv[k];
               seedsY[k] = sigmay[k] * inv[k];
            }
         }
      }
   }

   private static int enforceLabelConnectivity(int[] intputLabels, int width, int height, int[] newLabels, int numberOfLabels, int k)
   {
      int size = width * height;
      for (int i = 0; i < size; i++)
         newLabels[i] = -1;

      int SUPSZ = size / k;
      //------------------
      // labeling
      //------------------
      int lab = 0;
      int i = 0;
      int adjlabel = 0;//adjacent label
      int[] xvec = new int[size];//worst case size
      int[] yvec = new int[size];//worst case size
      int[] count = new int[1];
      for (int h = 0; h < height; h++)
      {
         for (int w = 0; w < width; w++)
         {
            if (newLabels[i] < 0)
            {
               newLabels[i] = lab;
               //-------------------------------------------------------
               // Quickly find an adjacent label for use later if needed
               //-------------------------------------------------------
               {
                  for (int n = 0; n < 4; n++)
                  {
                     int x = w + dx4[n];
                     int y = h + dy4[n];
                     if ((x >= 0 && x < width) && (y >= 0 && y < height))
                     {
                        int nindex = y * width + x;
                        if (newLabels[nindex] >= 0)
                        {
                           adjlabel = newLabels[nindex];
                        }
                     }
                  }
               }
               xvec[0] = w;
               yvec[0] = h;

               count[0] = 1;

               List<Integer> h_stack = new ArrayList<Integer>();
               List<Integer> w_stack = new ArrayList<Integer>();
               h_stack.add(h);
               w_stack.add(w);

               int s, x, y, h1, w1, ind;
               while (!h_stack.isEmpty())
               {
                  s = h_stack.size() - 1;
                  h1 = h_stack.remove(s);
                  w1 = w_stack.remove(s);
                  for (int i1 = 0; i1 < 4; i1++)
                  {
                     y = h1 + dy4[i1];
                     x = w1 + dx4[i1];
                     if ((y < height && y >= 0) && (x < width && x >= 0))
                     {
                        ind = y * width + x;
                        if (newLabels[ind] < 0 && intputLabels[ind] == intputLabels[h1 * width + w1])
                        {
                           xvec[count[0]] = x;
                           yvec[count[0]] = y;
                           count[0]++;
                           newLabels[ind] = lab;
                           h_stack.add(y);
                           w_stack.add(x);
                        }
                     }
                  }
               }

               //-------------------------------------------------------
               // If segment size is less then a limit, assign an
               // adjacent label found before, and decrement label count.
               //-------------------------------------------------------
               if (count[0] <= (SUPSZ >> 2))
               {
                  for (int c = 0; c < count[0]; c++)
                  {
                     int ind1 = yvec[c] * width + xvec[c];
                     newLabels[ind1] = adjlabel;
                  }
                  lab--;
               }
               lab++;
            }
            i++;
         }
      }
      numberOfLabels = lab;
      return numberOfLabels;
   }

   public static void performSLICOForGivenK(int[] imgBuffer, int width, int height, int[] kLabels, int numberOfLabels, int k, double m)
   {
      int size = width * height;
      for (int s = 0; s < size; s++)
         //kLabels[s] = -1;
         kLabels[s] = 0;

      double[] kseedsl = new double[size];
      double[] kseedsa = new double[size];
      double[] kseedsb = new double[size];
      double[] kseedsx = new double[size];
      double[] kseedsy = new double[size];

      double[] edgemag = new double[size];

      m_lvec = new double[size];
      m_avec = new double[size];
      m_bvec = new double[size];

      boolean useLABColor = false;
      if (useLABColor)
      {
         convertRGB2LAB(imgBuffer, width, height, m_lvec, m_avec, m_bvec);
      }
      else
      {
         for (int i = 0; i < size; i++)
         {
            m_lvec[i] = imgBuffer[i] >> 16 & 0xFF;
            m_avec[i] = imgBuffer[i] >> 8 & 0xFF;
            m_bvec[i] = imgBuffer[i] & 0xFF;
         }
      }

      System.out.println("# convertRGB2LAB is done ");

      boolean perturbseeds = true;

      if (perturbseeds)
         detectLABEdges(m_lvec, m_avec, m_bvec, width, height, edgemag);
      System.out.println("# detectLABEdges is done ");

      getLABXYSeedsForGivenK(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, width, height, k, perturbseeds, edgemag);
      System.out.println("# getLABXYSeedsForGivenStepSize is done ");

      int stepSize = (int) (Math.sqrt((double) (size) / (double) (k)) + 2); //adding a small value in the even the STEP size is too small.
      performSuperPixelSegmentationVariableSAndM(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, width, height, kLabels, stepSize, 10);
      System.out.println("# performSuperPixelSegmentationVariableSAndM is done ");

      numberOfLabels = kseedsl.length;

      int[] nlabels = new int[size];
      enforceLabelConnectivity(kLabels, width, height, nlabels, numberOfLabels, k);
      System.out.println("# enforceLabelConnectivity is done ");

      for (int i = 0; i < size; i++)
         kLabels[i] = nlabels[i];
   }

   public static void drawContoursAroundSegments(int[] imgBuffer, int[] kLabels, int width, int height, int contourColor)
   {
      int dx8[] = {-1, -1, 0, 1, 1, 1, 0, -1};
      int dy8[] = {0, -1, -1, -1, 0, 1, 1, 1};

      int sz = width * height;

      boolean[] istaken = new boolean[sz];
      for (int i = 0; i < sz; i++)
         istaken[i] = false;

      int mainindex = 0;
      for (int j = 0; j < height; j++)
      {
         for (int k = 0; k < width; k++)
         {
            int np = 0;
            for (int i = 0; i < 8; i++)
            {
               int x = k + dx8[i];
               int y = j + dy8[i];

               if ((x >= 0 && x < width) && (y >= 0 && y < height))
               {
                  int index = y * width + x;

                  if (false == istaken[index])//comment this to obtain internal contours
                  {
                     if (kLabels[mainindex] != kLabels[index])
                        np++;
                  }
               }
            }
            if (np > 1)//change to 2 or 3 for thinner lines
            {
               imgBuffer[mainindex] = contourColor;
               istaken[mainindex] = true;
            }
            mainindex++;
         }
      }
   }
}
