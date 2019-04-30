package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * Standard plane equation form.
 * a*x + b*y + c*z + d = 0.
 */
public class PlaneEquation
{
   private final Vector3D normalVector = new Vector3D();
   private double constantD = 0.0;

   /**
    * a*x + b*y + c*z + d = 0.
    * @param normal: a, b, c
    * @param constant: d
    */
   public PlaneEquation(Vector3D normal, double constant)
   {
      normalVector.set(normal);
      constantD = constant;
   }

   public PlaneEquation(Tuple3DBasics point1, Tuple3DBasics point2, Tuple3DBasics point3)
   {
      Vector3D vector1To2 = new Vector3D(point2);
      vector1To2.sub(point1);
      Vector3D vector1To3 = new Vector3D(point3);
      vector1To3.sub(point1);

      normalVector.cross(vector1To2, vector1To3);
      constantD = -normalVector.getX() * point1.getX() - normalVector.getY() * point1.getY() - normalVector.getZ() * point1.getZ();
   }

   public double distance(Tuple3DBasics point)
   {
      return Math.abs(normalVector.getX() * point.getX() + normalVector.getY() * point.getY() + normalVector.getZ() * point.getZ() + constantD)
            / Math.sqrt(normalVector.lengthSquared());
   }
}
