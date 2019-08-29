package us.ihmc.pathPlanning.simulation;

public class MultiSenseParameters extends RGBDSensorDescription
{
   private static final double widthFieldOfView = Math.toRadians(80);
   private static final double heightFieldOfView = Math.toRadians(45);

   //   private static final int pixelsWide = 2048;
   //   private static final int pixelsHigh = 1088;
   private static final int pixelsWide = 200;
   private static final int pixelsHigh = 100;

   private static final double minimumRange = 0.0;

   public MultiSenseParameters()
   {
      super(widthFieldOfView, heightFieldOfView, pixelsWide, pixelsHigh, minimumRange);
   }
}
