package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedPlanning.FancyQuadrupedXGaitSettingsReadOnly;

public class GenericQuadrupedFancyXGaitSettings implements FancyQuadrupedXGaitSettingsReadOnly
{
   private static final double stanceLength = 1.1;
   private static final double stanceWidth = 0.2;
   private static final double stepGroundClearance = 0.1;
   private static final double endPhaseShift = 90.0;

   private static final double paceStepDuration = 0.2;
   private static final double trotStepDuration = 0.33;
   private static final double crawlStepDuration = 0.5;

   private static final double paceDoubleSupportFraction = 0.2;
   private static final double trotDoubleSupportFraction = 0.3;
   private static final double crawlDoubleSupportFraction = 0.75;

   private static final double endDoubleSupportDuration = 0.1;
   private static final boolean useFractionalDoubleSupport = true;

   @Override
   public double getStanceLength()
   {
      return stanceLength;
   }

   @Override
   public double getStanceWidth()
   {
      return stanceWidth;
   }

   @Override
   public double getEndPhaseShift()
   {
      return endPhaseShift;
   }

   @Override
   public double getStepGroundClearance()
   {
      return stepGroundClearance;
   }

   @Override
   public double getPaceStepDuration()
   {
      return paceStepDuration;
   }

   @Override
   public double getCrawlStepDuration()
   {
      return crawlStepDuration;
   }

   @Override
   public double getTrotStepDuration()
   {
      return trotStepDuration;
   }

   @Override
   public double getPaceDoubleSupportFraction()
   {
      return paceDoubleSupportFraction;
   }

   @Override
   public double getCrawlDoubleSupportFraction()
   {
      return crawlDoubleSupportFraction;
   }

   @Override
   public double getTrotDoubleSupportFraction()
   {
      return trotDoubleSupportFraction;
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return endDoubleSupportDuration;
   }

   @Override
   public boolean useFractionalDoubleSupport()
   {
      return useFractionalDoubleSupport;
   }
}
