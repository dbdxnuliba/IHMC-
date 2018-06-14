package us.ihmc.avatar.collinearVisualizer;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class CentroidalRobotPhysicalProperties
{
   public abstract double getMass();

   public abstract double getNominalHeight();

   public abstract SideDependentList<ConvexPolygon2D> getDefaultSupportPolygons();

   public abstract DenseMatrix64F getInertia();
}