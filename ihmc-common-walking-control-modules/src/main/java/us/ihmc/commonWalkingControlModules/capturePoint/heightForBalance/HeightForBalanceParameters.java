package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

public abstract class HeightForBalanceParameters
{
   public abstract double getMaxHeightFirstPhaseOfSwing();

   public abstract double getMinHeight();

   public double getMaxHeightFractionForSecondPhaseOfSwing()
   {
      return 0.97;
   }


   public double getMaxHeightAccelerationForAngleCase()
   {
      return 5;
   }

   public double getMinHeightAccelerationForAngleCase()
   {
      return -5;
   }

   public double getFractionOfMaxHeightAccelerationToConsiderInPrediction()
   {
      return 0.4;
   }

   public double getMaxHeightAccelerationForDistanceCase()
   {
      return 3.0;
   }

   public double getMinHeightAccelerationForDistanceCase()
   {
      return -1.0;
   }

   public double getMaximumJerk()
   {
      return 200;
   }

   public double getAnglePositiveAlignmentThresholdFromStart()
   {
      return 0.8;
   }

   public double getAngleNegativeAlignmentThreshold()
   {
      return Math.PI - 1.0;
   }

   public double getMinimumCoPCoMProjectedICPeDistanceToControl()
   {
      return 0.04;
   }


   public double getFractionOfSwingTimeToChangeMaxHeight()
   {
      return 0.416;
   }

}
