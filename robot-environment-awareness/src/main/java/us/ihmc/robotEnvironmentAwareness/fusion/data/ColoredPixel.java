package us.ihmc.robotEnvironmentAwareness.fusion.data;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ColoredPixel
{
   private final Point3DReadOnly point;
   private final int color;

   public ColoredPixel(Point3DReadOnly point, int color)
   {
      this.point = point;
      this.color = color;
   }

   public Point3DReadOnly getPoint()
   {
      return point;
   }

   public int getColor()
   {
      return color;
   }
}
