package us.ihmc.robotics.math;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code Pose3DBasics} implementation which position and orientation baked with
 * {@code YoVariable}s.
 */
public class YoPose3D implements Pose3DBasics, GeometryObject<YoPose3D>
{
   /** The position part of this pose 3D. */
   private final YoPoint3D position;
   /** The orientation part of this pose 3D. */
   private final YoQuaternion orientation;

   /**
    * Creates a new {@code YoPose3D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoPose3D(String namePrefix, YoVariableRegistry registry)
   {
      position = new YoPoint3D(String.format("%s_point", namePrefix), registry);
      orientation = new YoQuaternion(String.format("%s_orientation", namePrefix), registry);
   }

   /**
    * Creates a new {@code YoPose3D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoPose3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      position = new YoPoint3D(String.format("%s_point", namePrefix), nameSuffix, registry);
      orientation = new YoQuaternion(String.format("%s_orientation", namePrefix), nameSuffix, registry);
   }

   @Override
   public void set(YoPose3D other)
   {
      Pose3DBasics.super.set(other);
   }

   @Override
   public void setX(double x)
   {
      position.setX(x);
   }

   @Override
   public void setY(double y)
   {
      position.setY(y);
   }

   @Override
   public void setZ(double z)
   {
      position.setZ(z);
   }

   @Override
   public Point3DBasics getPosition()
   {
      return position;
   }

   @Override
   public QuaternionBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public double getX()
   {
      return position.getX();
   }

   @Override
   public double getY()
   {
      return position.getY();
   }

   @Override
   public double getZ()
   {
      return position.getZ();
   }

   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   @Override
   public double getPitch()
   {
      return orientation.getPitch();
   }

   @Override
   public double getRoll()
   {
      return orientation.getRoll();
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((Pose3DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(YoPose3D other, double epsilon)
   {
      return Pose3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(YoPose3D other, double epsilon)
   {
      return Pose3DBasics.super.geometricallyEquals(other, epsilon);
   }
}
