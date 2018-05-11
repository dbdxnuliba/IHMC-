package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

/**
 * The hyper parameters for the SQP running in the {@code CollinearForceBasedCoMMotionPlanner}
 * Should be extended as required for individual robots
 * @author apoorv
 *
 */
public class CollinearForcePlannerParameters
{
   public int getMaxSQPIterations()
   {
      return 10;
   }

   public double getMaxPlannerSegmentTime()
   {
      return 0.05;
   }

   public double getMinPlannerSegmentTime()
   {
      return 0.01;
   }

   public int getNumberOfContactStatesToPlan()
   {
      return 60;
   }

   public double getConsolidatedConvergenceThreshold()
   {
      return 3e-5;
   }

   public double getIndividualAxisConvergenceThreshold()
   {
      return 1e-5;
   }
}
