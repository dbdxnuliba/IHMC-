package us.ihmc.robotics.math;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DBasics;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code Orientation2DBasics} implementation which position and orientation baked with
 * {@code YoVariable}s.
 */
public class YoOrientation2D implements Orientation2DBasics, GeometryObject<YoOrientation2D>
{
   /** The angle in radians about the z-axis. */
   private final YoDouble yaw;

   /** Orientation used to transform {@code this} in {@link #applyTransform(Transform)}. */
   private final Orientation2D orientation = new Orientation2D();

   /**
    * Creates a new {@code YoOrientation2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoOrientation2D(String namePrefix, YoVariableRegistry registry)
   {
      yaw = new YoDouble(String.format("%s_yaw", namePrefix), registry);
   }

   /**
    * Creates a new {@code YoOrientation2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoOrientation2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      yaw = new YoDouble(String.format("%s_yaw_%s", namePrefix, nameSuffix), registry);
   }

   @Override
   public void set(YoOrientation2D other)
   {
      Orientation2DBasics.super.set(other);
   }

   @Override
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw);
   }

   @Override
   public double getYaw()
   {
      return yaw.getDoubleValue();
   }

   @Override
   public boolean epsilonEquals(YoOrientation2D other, double epsilon)
   {
      return Orientation2DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(YoOrientation2D other, double epsilon)
   {
      return Orientation2DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      orientation.set(this);
      orientation.applyTransform(transform);
      set(orientation);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.set(this);
      orientation.applyInverseTransform(transform);
      set(orientation);
   }
}
