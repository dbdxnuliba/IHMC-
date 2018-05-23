package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * The hyper parameters for the SQP running in the {@code CollinearForceBasedCoMMotionPlanner}
 * Should be extended as required for individual robots
 * @author Apoorv S
 *
 */
public class CollinearForcePlannerParameters
{
   public int getMaxSQPIterations()
   {
      return 10;
   }

   public double getNominalPlannerSegmentTime()
   {
      return 0.1;
   }

   public double getMinPlannerSegmentTime()
   {
      return 0.05;
   }

   public int getNumberOfContactStatesToPlan()
   {
      return 6;
   }

   public double getConsolidatedConvergenceThreshold()
   {
      return 3e-5;
   }

   public double getIndividualAxisConvergenceThreshold()
   {
      return 1e-5;
   }

   public Point3DReadOnly getNominalCoMOffsetFromSupportPolygonCentroid()
   {
      return new Point3D(0.0, 0.0, 0.435);
   }

   public double getSupportPolygonMXYOffsetForCoMConstraint()
   {
      return 0.10;
   }

   public double getMaxZHeight()
   {
      return 0.45;
   }
   
   public double getMinZHeight()
   {
      return 0.15;
   }

   public int getNumberOfCollocationConstraintsPerSegment()
   {
      return 2;
   }

   public int getNumberOfCoMPositionConstraintsPerSegment()
   {
      return 2;
   }

   public int getNumberOfSupportPolygonConstraintsPerSegment()
   {
      return 4;
   }

   public int getNumberOfScalarConstraintsPerSegment()
   {
      return 4;
   }
}
