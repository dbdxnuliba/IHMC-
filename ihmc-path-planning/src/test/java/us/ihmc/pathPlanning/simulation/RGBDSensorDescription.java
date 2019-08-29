package us.ihmc.pathPlanning.simulation;

public class RGBDSensorDescription
{
   private final double widthFieldOfView;
   private final double heightFieldOfView;

   private final int pixelsWide;
   private final int pixelsHigh;

   private final double minimumRange;

   public RGBDSensorDescription(double widthFieldOfView, double heightFieldOfView, int pixelsWide, int pixelsHigh, double minimumRange)
   {
      this.widthFieldOfView = widthFieldOfView;
      this.heightFieldOfView = heightFieldOfView;
      this.pixelsHigh = pixelsHigh;
      this.pixelsWide = pixelsWide;
      this.minimumRange = minimumRange;
   }

   public double getWidthFieldOfView()
   {
      return widthFieldOfView;
   }

   public double getHeightFieldOfView()
   {
      return heightFieldOfView;
   }

   public int getPixelsWide()
   {
      return pixelsWide;
   }

   public int getPixelsHigh()
   {
      return pixelsHigh;
   }

   public double getMinimumRange()
   {
      return minimumRange;
   }
}
