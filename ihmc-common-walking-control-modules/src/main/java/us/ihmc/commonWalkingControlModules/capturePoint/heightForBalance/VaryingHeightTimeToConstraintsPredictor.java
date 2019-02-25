package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import static java.lang.Math.sqrt;

public class VaryingHeightTimeToConstraintsPredictor
{
   private final double zMin;

   public VaryingHeightTimeToConstraintsPredictor(double zMin)
   {
      this.zMin = zMin;

   }

   public double getTMinPosReachedPredicted(double zCurrent, double dzCurrent, double aMinPredicted, double aMaxPredicted)
   {
      double zMinForPrediction = 1.03 * zMin;
      double a = 0.5 * (aMinPredicted - aMinPredicted * aMinPredicted / aMaxPredicted);
      double b = dzCurrent - dzCurrent * aMinPredicted / aMaxPredicted;
      double c = zCurrent - zMinForPrediction - 0.5 * dzCurrent * dzCurrent / aMaxPredicted;
      double tMinPosReachedPredicted = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      tMinPosReachedPredicted = Math.max(0, tMinPosReachedPredicted);
      if (Double.isNaN(tMinPosReachedPredicted))
      {
         tMinPosReachedPredicted = 0;
      }
      return tMinPosReachedPredicted;
   }

   public double getTMaxPosReachedPredicted(double zCurrent, double dzCurrent, double zMax, double aMinPredicted, double aMaxPredicted)
   {
      double zMaxForPrediction = zMax;
      double a = 0.5 * (aMaxPredicted + aMaxPredicted * aMaxPredicted / -aMinPredicted);
      double b = dzCurrent + dzCurrent * aMaxPredicted / -aMinPredicted;
      double c = zCurrent - zMaxForPrediction + 0.5 * dzCurrent * dzCurrent / -aMinPredicted;
      double tMaxPosReachedPredicted = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      tMaxPosReachedPredicted = Math.max(0, tMaxPosReachedPredicted);
      if (Double.isNaN(tMaxPosReachedPredicted))
      {
         tMaxPosReachedPredicted = 0;
      }
      return tMaxPosReachedPredicted;
   }

   public double getTMinVelReachedPredicted(double dzCurrent, double aMin, double vMin)
   {
      double tMinVelReachedPredicted = (vMin - dzCurrent) / aMin;
      tMinVelReachedPredicted = Math.max(0, tMinVelReachedPredicted);
      if (Double.isNaN(tMinVelReachedPredicted))
      {
         tMinVelReachedPredicted = 0;
      }
      return tMinVelReachedPredicted;
   }

   public double getTMaxVelReachedPredicted(double dzCurrent, double aMax, double vMax)
   {
      double tMaxVelReachedPredicted = (vMax - dzCurrent) / aMax;
      tMaxVelReachedPredicted = Math.max(0, tMaxVelReachedPredicted);
      if (Double.isNaN(tMaxVelReachedPredicted))
      {
         tMaxVelReachedPredicted = 0;
      }
      return tMaxVelReachedPredicted;
   }

}