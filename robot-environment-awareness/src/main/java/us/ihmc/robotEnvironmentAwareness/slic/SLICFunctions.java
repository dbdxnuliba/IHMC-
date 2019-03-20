package us.ihmc.robotEnvironmentAwareness.slic;

public final class SLICFunctions
{
   private int[] dx4 = {-1, 0, 1, 0};
   private int[] dy4 = {0, -1, 0, 1};

   private int m_width;
   private int m_height;

   private double[] m_lvec = null;
   private double[] m_avec = null;
   private double[] m_bvec = null;

   private double[] m_lvecvec = null;
   private double[] m_avecvec = null;
   private double[] m_bvecvec = null;

   private int[] dx10 = {-1, 0, 1, 0, -1, 1, 1, -1, 0, 0};
   private int[] dy10 = {0, -1, 0, 1, -1, -1, 1, 1, 0, 0};
   private int[] dz10 = {0, 0, 0, 0, 0, 0, 0, 0, -1, 1};

   public static void convertRGB2XYZ(int r, int g, int b, double xToPack, double yToPack, double zToPack)
   {

   }

   public static void convertRGB2LAB(int r, int g, int b, double lToPack, double aToPack, double bToPack)
   {

   }

   public static void convertRGB2LAB(int[] imgBuffer, double[] lBufferToPack, double[] aBufferToPack, double[] bBufferToPack)
   {

   }

   public static void detectLABEdges(double[] lBuffer, double[] aBuffer, double[] bBuffer, int width, int height, double[] edges)
   {

   }

   public static void perturbSeeds(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, double[] edges)
   {

   }

   public static void getLABXYSeedsForGivenStepSize(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, int step,
                                                    int perturbSeeds, double[] edgeMagnitude)
   {

   }

   public static void getLABXYSeedsForGivenK(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY, int k, int perturbSeeds,
                                             double[] edgeMagnitude)
   {

   }

   public static void performSuperPixelSegmentationVariableSAndM(double[] seedsL, double[] seedsA, double[] seedsB, double[] seedsX, double[] seedsY,
                                                                 int[] kLabels, int step, int numberOfIteration)
   {

   }

   public static void enforceLabelConnectivity(int[] intputLabels, int width, int height, int[] newLabels, int numberOfLabels, int k)
   {

   }

   public static void performSLICOForGivenStepSize(int[] imgBuffer, int width, int height, int[] kLabels, int numerOfLabels, int step, double m)
   {

   }

   public static void performSLICOForGivenK(int[] imgBuffer, int width, int height, int[] kLabels, int numerOfLabels, int k, double m)
   {

   }
}
