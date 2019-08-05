package us.ihmc.robotEnvironmentAwareness.planarRegion;

public class StereoFilterParameters
{
   private static final int DEFAULT_NUMBER_OF_BUFFER = 1;
   private static final int DEFAULT_SIZE_OF_BUFFER = 100000;
   private static final double DEFAULT_SURFACE_NORMAL_UPPER_BOUND_DEGREE = 40.0;
   private static final double DEFAULT_SURFACE_NORMAL_LOWER_BOUND_DEGREE = -40.0;

   private int numberOfBuffer;
   private int sizeOfBuffer;
   private double surfaceNormalUpperBoundDegree;
   private double surfaceNormalLowerBoundDegree;

   public StereoFilterParameters()
   {
      setDefaultParameters();
   }

   public StereoFilterParameters(StereoFilterParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      numberOfBuffer = DEFAULT_NUMBER_OF_BUFFER;
      sizeOfBuffer = DEFAULT_SIZE_OF_BUFFER;
      surfaceNormalUpperBoundDegree = DEFAULT_SURFACE_NORMAL_UPPER_BOUND_DEGREE;
      surfaceNormalLowerBoundDegree = DEFAULT_SURFACE_NORMAL_LOWER_BOUND_DEGREE;
   }

   public void set(StereoFilterParameters other)
   {
      numberOfBuffer = other.numberOfBuffer;
      sizeOfBuffer = other.sizeOfBuffer;
      surfaceNormalUpperBoundDegree = other.surfaceNormalUpperBoundDegree;
      surfaceNormalLowerBoundDegree = other.surfaceNormalLowerBoundDegree;
   }

   @Override
   public String toString()
   {
      return "numberOfBuffer: " + numberOfBuffer + ", sizeOfBuffer: " + sizeOfBuffer + ", surfaceNormalUpperBoundDegree: " + surfaceNormalUpperBoundDegree
            + ", surfaceNormalLowerBoundDegree: " + surfaceNormalLowerBoundDegree;
   }

   public int getNumberOfBuffer()
   {
      return numberOfBuffer;
   }

   public int getSizeOfBuffer()
   {
      return sizeOfBuffer;
   }

   public double getSurfaceNormalUpperBoundDegree()
   {
      return surfaceNormalUpperBoundDegree;
   }

   public double getSurfaceNormalLowerBoundDegree()
   {
      return surfaceNormalLowerBoundDegree;
   }

   public void setNumberOfBuffer(int numberOfBuffer)
   {
      this.numberOfBuffer = numberOfBuffer;
   }

   public void setSizeOfBuffer(int sizeOfBuffer)
   {
      this.sizeOfBuffer = sizeOfBuffer;
   }

   public void setSurfaceNormalUpperBoundDegree(double surfaceNormalUpperBoundDegree)
   {
      this.surfaceNormalUpperBoundDegree = surfaceNormalUpperBoundDegree;
   }

   public void setSurfaceNormalLowerBoundDegree(double surfaceNormalLowerBoundDegree)
   {
      this.surfaceNormalLowerBoundDegree = surfaceNormalLowerBoundDegree;
   }

}
