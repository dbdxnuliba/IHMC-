package us.ihmc.quadrupedPlanning;

public class FancyQuadrupedXGaitSettings implements FancyQuadrupedXGaitSettingsReadOnly
{
   private double stanceLength;
   private double stanceWidth;
   private double stepGroundClearance;
   private double endPhaseShift;

   private double paceStepDuration;
   private double crawlStepDuration;
   private double trotStepDuration;

   private double paceDoubleSupportFraction;
   private double trotDoubleSupportFraction;
   private double crawlDoubleSupportFraction;

   private double endDoubleSupportDuration;
   private boolean useFractionalDoubleSupport;

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

   public void setStanceLength(double stanceLength)
   {
      this.stanceLength = stanceLength;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public void setStepGroundClearance(double stepGroundClearance)
   {
      this.stepGroundClearance = stepGroundClearance;
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      this.endPhaseShift = endPhaseShift;
   }

   public void setPaceStepDuration(double paceStepDuration)
   {
      this.paceStepDuration = paceStepDuration;
   }

   public void setCrawlStepDuration(double crawlStepDuration)
   {
      this.crawlStepDuration = crawlStepDuration;
   }

   public void setTrotStepDuration(double trotStepDuration)
   {
      this.trotStepDuration = trotStepDuration;
   }

   public void setPaceDoubleSupportFraction(double paceDoubleSupportFraction)
   {
      this.paceDoubleSupportFraction = paceDoubleSupportFraction;
   }

   public void setCrawlDoubleSupportFraction(double crawlDoubleSupportFraction)
   {
      this.crawlDoubleSupportFraction = crawlDoubleSupportFraction;
   }

   public void setTrotDoubleSupportFraction(double trotDoubleSupportFraction)
   {
      this.trotDoubleSupportFraction = trotDoubleSupportFraction;
   }

   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      this.endDoubleSupportDuration = endDoubleSupportDuration;
   }

   public void setUseFractionalDoubleSupport(boolean useFractionalDoubleSupport)
   {
      this.useFractionalDoubleSupport = useFractionalDoubleSupport;
   }

   public void set(FancyQuadrupedXGaitSettingsReadOnly other)
   {
      setStanceLength(other.getStanceLength());
      setStanceWidth(other.getStanceWidth());
      setStepGroundClearance(other.getStepGroundClearance());
      setEndPhaseShift(other.getEndPhaseShift());

      setPaceStepDuration(other.getPaceStepDuration());
      setCrawlStepDuration(other.getCrawlStepDuration());
      setTrotStepDuration(other.getTrotStepDuration());

      setPaceDoubleSupportFraction(other.getPaceDoubleSupportFraction());
      setTrotDoubleSupportFraction(other.getTrotDoubleSupportFraction());
      setCrawlDoubleSupportFraction(other.getCrawlDoubleSupportFraction());

      setEndDoubleSupportDuration(other.getEndDoubleSupportDuration());
      setUseFractionalDoubleSupport(other.useFractionalDoubleSupport());
   }
}
