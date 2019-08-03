package us.ihmc.robotEnvironmentAwareness.fusion.data;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class StereoPoint extends Point3D
{
   private final int rgbColor;
   private final int[] pixelIndices;

   public StereoPoint(Point3DReadOnly point, int color, int[] pixelIndices)
   {
      super(point);

      this.rgbColor = color;
      this.pixelIndices = pixelIndices;
   }

   public int[] getPixelIndices()
   {
      return pixelIndices;
   }

   public int getColor()
   {
      return rgbColor;
   }

   public int getXIndex()
   {
      return pixelIndices[0];
   }

   public int getYIndex()
   {
      return pixelIndices[1];
   }
}
