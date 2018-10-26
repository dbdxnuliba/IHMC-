package us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance;

import static java.lang.Math.sqrt;

public class VaryingHeightTimeToConstraintsPredictor
{
   private final double zMin;
   private final double vMin;
   private final double vMax;
   private final double aMinPredicted;
   private final double aMaxPredicted;

   /**
    * Gives an estimate of the time until the maximum velocity or position constraint is reached, given a predicted constant acceleration and deceleration.
    * Those are a predefined fraction of the min/max acceleration for the controller.
    * @param zMin
    * @param vMin
    * @param vMax
    * @param aMinPredicted
    * @param aMaxPredicted
    */
   public VaryingHeightTimeToConstraintsPredictor(double zMin, double vMin, double vMax, double aMinPredicted, double aMaxPredicted)
   {
      this.zMin=zMin;
      this.vMin=vMin;
      this.vMax=vMax;
      this.aMinPredicted=aMinPredicted;
      this.aMaxPredicted=aMaxPredicted;
   }

   /**
    * Predicted time to minimum position, future velocity incorporated
    * @param zCurrent current height
    * @param dzCurrent current height velocity
    * @return
    */
   public double getTMinPosReachedPredicted(double zCurrent, double dzCurrent)
   {
      double zMinForPrediction = 1.03 * zMin;
      double a = 0.5 * (aMinPredicted - aMinPredicted * aMinPredicted / aMaxPredicted);
      double b = dzCurrent - dzCurrent * aMinPredicted / aMaxPredicted;
      double c = zCurrent - zMinForPrediction - 0.5 * dzCurrent * dzCurrent / aMaxPredicted;
      double tMinPosReachedPredicted = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      tMinPosReachedPredicted = Math.max(0, tMinPosReachedPredicted);
      return tMinPosReachedPredicted;
   }

   /**
    * Predicted time to maximum position, future velocity incorporated
    * @param zCurrent
    * @param dzCurrent
    * @param zMax Max height, changes halfway swing
    * @return
    */
   public double getTMaxPosReachedPredicted(double zCurrent, double dzCurrent, double zMax)
   {
      double zMaxForPrediction = zMax;
      double a = 0.5 * (aMaxPredicted + aMaxPredicted * aMaxPredicted / -aMinPredicted);
      double b = dzCurrent + dzCurrent * aMaxPredicted / -aMinPredicted;
      double c = zCurrent - zMaxForPrediction + 0.5 * dzCurrent * dzCurrent / -aMinPredicted;
      double tMaxPosReachedPredicted = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      tMaxPosReachedPredicted = Math.max(0, tMaxPosReachedPredicted);
      return tMaxPosReachedPredicted;
   }

   /**
    * Predicted time to minimum velocity
    * @param dzCurrent
    * @return
    */
   public double getTMinVelReachedPredicted(double dzCurrent)
   {
      double tMinVelReachedPredicted = (vMin - dzCurrent) / aMinPredicted;
      tMinVelReachedPredicted = Math.max(0, tMinVelReachedPredicted);
      return tMinVelReachedPredicted;
   }
   public double getTMaxVelReachedPredicted(double dzCurrent)
   {
      double tMaxVelReachedPredicted = (vMax - dzCurrent) / aMaxPredicted;
      tMaxVelReachedPredicted = Math.max(0, tMaxVelReachedPredicted);
      return tMaxVelReachedPredicted;
   }

}
