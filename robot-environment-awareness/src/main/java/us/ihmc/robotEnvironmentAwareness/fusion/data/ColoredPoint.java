package us.ihmc.robotEnvironmentAwareness.fusion.data;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ColoredPoint extends Point3D
{
   private final int color;

   public ColoredPoint(int color)
   {
      this.color = color;
   }

   public ColoredPoint(Point3DReadOnly point, int color)
   {
      super(point);
      this.color = color;
   }

   public int getColor()
   {
      return color;
   }
}
