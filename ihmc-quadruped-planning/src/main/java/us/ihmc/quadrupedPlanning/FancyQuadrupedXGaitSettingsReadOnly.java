package us.ihmc.quadrupedPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.AngleTools;

import static java.lang.Math.PI;

public interface FancyQuadrupedXGaitSettingsReadOnly
{
   static final double pacePhase = 0.0;
   static final double crawlPhase = 90.0;
   static final double trotPhase = 180.0;

   /**
    * Nominal x offset between front and hind feet (in meters).
    */
   double getStanceLength();

   /**
    * Nominal y offset between left and right feet (in meters).
    */
   double getStanceWidth();

   /**
    * Nominal phase shift between front and hind steps (in degrees, 0: pace, 90: amble, 180: trot).
    */
   double getEndPhaseShift();

   /**
    * Ground clearance for each step (in meters).
    */
   double getStepGroundClearance();


   double getPaceStepDuration();
   double getCrawlStepDuration();
   double getTrotStepDuration();

   default double getStepDuration()
   {
      double phaseForMath = Math.abs(Math.toRadians(getEndPhaseShift()));
      double alpha = 2.0 * phaseForMath / Math.PI;

      double initialCondition;
      double finalCondition;
      if (phaseForMath < Math.PI / 2.0)
      {
         initialCondition = getPaceStepDuration();
         finalCondition = getCrawlStepDuration();
      }
      else
      {
         initialCondition = getCrawlStepDuration();
         finalCondition = getTrotStepDuration();
         alpha -= 1.0;
      }

      return InterpolationTools.hermiteInterpolate(initialCondition, finalCondition, alpha);
   }

   double getPaceDoubleSupportFraction();
   double getCrawlDoubleSupportFraction();
   double getTrotDoubleSupportFraction();

   default double getDoubleSupportFraction()
   {
      double phaseForMath = Math.abs(Math.toRadians(getEndPhaseShift()));
      double alpha = 2.0 * phaseForMath / Math.PI;

      double initialCondition;
      double finalCondition;
      if (phaseForMath < Math.PI / 2.0)
      {
         initialCondition = getPaceDoubleSupportFraction();
         finalCondition = getCrawlDoubleSupportFraction();
      }
      else
      {
         initialCondition = getCrawlDoubleSupportFraction();
         finalCondition = getTrotDoubleSupportFraction();
         alpha -= 1.0;
      }

      return InterpolationTools.hermiteInterpolate(initialCondition, finalCondition, alpha);
   }

   boolean useFractionalDoubleSupport();
   double getEndDoubleSupportDuration();

   default boolean epsilonEquals(FancyQuadrupedXGaitSettingsReadOnly other, double epsilon)
   {
      boolean equals = MathTools.epsilonEquals(this.getStanceLength(), other.getStanceLength(), epsilon);
      equals &= MathTools.epsilonEquals(this.getStanceWidth(), other.getStanceWidth(), epsilon);
      equals &= MathTools.epsilonEquals(this.getStepGroundClearance(), other.getStepGroundClearance(), epsilon);
      equals &= MathTools.epsilonEquals(this.getEndPhaseShift(), other.getEndPhaseShift(), epsilon);
      equals &= MathTools.epsilonEquals(this.getPaceStepDuration(), other.getPaceStepDuration(), epsilon);
      equals &= MathTools.epsilonEquals(this.getCrawlStepDuration(), other.getCrawlStepDuration(), epsilon);
      equals &= MathTools.epsilonEquals(this.getTrotStepDuration(), other.getTrotStepDuration(), epsilon);
      equals &= MathTools.epsilonEquals(this.getPaceDoubleSupportFraction(), other.getPaceDoubleSupportFraction(), epsilon);
      equals &= MathTools.epsilonEquals(this.getCrawlDoubleSupportFraction(), other.getCrawlDoubleSupportFraction(), epsilon);
      equals &= MathTools.epsilonEquals(this.getTrotDoubleSupportFraction(), other.getTrotDoubleSupportFraction(), epsilon);
      return equals;
   }


   public static void main(String[] args)
   {
      PrintTools.info("" + AngleTools.trimAngleMinusPiToPi(1.9 * PI));
   }
}
