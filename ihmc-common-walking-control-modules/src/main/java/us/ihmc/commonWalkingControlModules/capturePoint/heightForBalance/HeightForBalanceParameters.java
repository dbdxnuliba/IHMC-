package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

public abstract class HeightForBalanceParameters
{
   public abstract double getMaxHeightFirstPhaseOfSwing();

   public abstract double getMinHeight();

   public double getMaxHeightFractionForSecondPhaseOfSwing()
   {
      return 0.97;
   }

   public double getMinKneeAngle()
   {
      return 0.10;
   }

   public double getMaxKneeAngle()
   {
      return 2.1;
   }

   public double getMaxHeightAccelerationForAngleCase()
   {
      return 2.4;
   }

   public double getMinHeightAccelerationForAngleCase()
   {
      return -2.4;
   }

   public double getFractionOfMaxHeightAccelerationToConsiderInPrediction()
   {
      return 0.9;
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
      return 80.0;
   }

   public double getAnglePositiveAlignmentThresholdFromStart()
   {
      return 0.7;
   }

   public double getAngleNegativeAlignmentThreshold()
   {
      return Math.PI - 1.0;
   }

   public double getMinimumCoPCoMProjectedICPeDistanceToControl()
   {
      return 0.04;
   }

   public double getMaxVelocityForPrediction()
   {
      return 0.7;
   }

   public double getMinVelocityForPrediction()
   {
      return -0.7;
   }

   public double getSmoothEpsilon()
   {
      return 0.015;
   }

   public double getFractionCMPOfMaxDistanceFromPolygonForHeightControl()
   {
      return 0.06;
   }

   public double getDesiredICPVelocityToConsiderForNonDynamicCase()
   {
      return 0.02;
   }

   public double getFractionOfSwingTimeToChangeMaxHeight()
   {
      return 0.416;
   }

}
