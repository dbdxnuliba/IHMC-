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

   private double swingDurationWeight;
   private double doubleSupportDurationWeight;
   private double stepLengthWeight;
   private double stepWidthWeight;
   private double nominalStepLength;
   private double nominalStepWidth;
   private double minimumSwingDuration;
   private double maximumSwingDuration;

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

   @Override
   public double getSwingDurationWeight()
   {
      return swingDurationWeight;
   }

   @Override
   public double getDoubleSupportDurationWeight()
   {
      return doubleSupportDurationWeight;
   }

   @Override
   public double getStepLengthWeight()
   {
      return stepLengthWeight;
   }

   @Override
   public double getStepWidthWeight()
   {
      return stepWidthWeight;
   }

   @Override
   public double getNominalStepLength()
   {
      return nominalStepLength;
   }

   @Override
   public double getNominalStepWidth()
   {
      return nominalStepWidth;
   }

   @Override
   public double getMinimumSwingDuration()
   {
      return minimumSwingDuration;
   }

   @Override
   public double getMaximumSwingDuration()
   {
      return maximumSwingDuration;
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

   public void setSwingDurationWeight(double swingDurationWeight)
   {
      this.swingDurationWeight = swingDurationWeight;
   }

   public void setDoubleSupportDurationWeight(double doubleSupportDurationWeight)
   {
      this.doubleSupportDurationWeight = doubleSupportDurationWeight;
   }

   public void setStepLengthWeight(double stepLengthWeight)
   {
      this.stepLengthWeight = stepLengthWeight;
   }

   public void setStepWidthWeight(double stepWidthWeight)
   {
      this.stepWidthWeight = stepWidthWeight;
   }

   public void setNominalStepLength(double nominalStepLength)
   {
      this.nominalStepLength = nominalStepLength;
   }

   public void setNominalStepWidth(double nominalStepWidth)
   {
      this.nominalStepWidth = nominalStepWidth;
   }

   public void setMinimumSwingDuration(double minimumSwingDuration)
   {
      this.minimumSwingDuration = minimumSwingDuration;
   }

   public void setMaximumSwingDuration(double maximumSwingDuration)
   {
      this.maximumSwingDuration = maximumSwingDuration;
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

      setSwingDurationWeight(other.getSwingDurationWeight());
      setDoubleSupportDurationWeight(other.getDoubleSupportDurationWeight());
      setStepLengthWeight(other.getStepLengthWeight());
      setStepWidthWeight(other.getStepWidthWeight());
      setNominalStepLength(other.getNominalStepLength());
      setNominalStepWidth(other.getNominalStepWidth());
      setMinimumSwingDuration(other.getMinimumSwingDuration());
      setMaximumSwingDuration(other.getMaximumSwingDuration());
   }
}
