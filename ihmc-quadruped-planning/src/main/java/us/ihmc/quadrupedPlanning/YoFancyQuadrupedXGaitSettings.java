package us.ihmc.quadrupedPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFancyQuadrupedXGaitSettings implements FancyQuadrupedXGaitSettingsReadOnly
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter stanceLengthLowerLimitParameter = new DoubleParameter("stanceLengthLowerLimit", registry, 0.4);
   private final DoubleParameter stanceLengthUpperLimitParameter = new DoubleParameter("stanceLengthUpperLimit", registry, 1.4);
   private final DoubleParameter stanceWidthLowerLimitParameter = new DoubleParameter("stanceWidthLowerLimit", registry, 0.1);
   private final DoubleParameter stanceWidthUpperLimitParameter = new DoubleParameter("stanceWidthUpperLimit", registry, 0.6);
   private final DoubleParameter stepGroundClearanceLowerLimitParameter = new DoubleParameter("stepGroundClearanceLowerLimit", registry, 0.0);
   private final DoubleParameter stepGroundClearanceUpperLimitParameter = new DoubleParameter("stepGroundClearanceUpperLimit", registry, 0.25);
   private final DoubleParameter stepDurationLowerLimitParameter = new DoubleParameter("stepDurationLowerLimit", registry, 0.15);
   private final DoubleParameter stepDurationUpperLimitParameter = new DoubleParameter("stepDurationUpperLimit", registry, 0.6);
   private final DoubleParameter endDoubleSupportDurationLowerLimitParameter = new DoubleParameter("endDoubleSupportDurationLowerLimit", registry, 0.0);
   private final DoubleParameter endDoubleSupportDurationUpperLimitParameter = new DoubleParameter("endDoubleSupportDurationUpperLimit", registry, Double.MAX_VALUE);
   private final DoubleParameter endPhaseShiftLowerLimitParameter = new DoubleParameter("endPhaseShiftLowerLimit", registry, 0);
   private final DoubleParameter endPhaseShiftUpperLimitParameter = new DoubleParameter("endPhaseShiftUpperLimit", registry, 359);

   private final YoDouble yoStanceLength = new YoDouble("stanceLengthInput", registry);
   private final YoDouble yoStanceWidth = new YoDouble("stanceWidthInput", registry);
   private final YoDouble yoStepGroundClearance = new YoDouble("stepGroundClearanceInput", registry);
   private final YoDouble yoEndPhaseShift = new YoDouble("endPhaseShiftInput", registry);

   private final YoDouble paceStepDuration = new YoDouble("paceStepDurationInput", registry);
   private final YoDouble crawlStepDuration = new YoDouble("crawlStepDurationInput", registry);
   private final YoDouble trotStepDuration = new YoDouble("trotStepDurationInput", registry);

   private final YoDouble paceDoubleSupportFraction = new YoDouble("paceDoubleSupportFraction", registry);
   private final YoDouble crawlDoubleSupportFraction = new YoDouble("crawlDoubleSupportFraction", registry);
   private final YoDouble trotDoubleSupportFraction = new YoDouble("trotDoubleSupportFraction", registry);

   private final YoDouble endDoubleSupportDuration = new YoDouble("endDoubleSupportDuration", registry);

   private final YoBoolean useFractionalDoubleSupport = new YoBoolean("useFractionalDoubleSupport", registry);

   public YoFancyQuadrupedXGaitSettings(FancyQuadrupedXGaitSettingsReadOnly defaultXGaitSettings, GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      yoStanceLength.set(defaultXGaitSettings.getStanceLength());
      yoStanceWidth.set(defaultXGaitSettings.getStanceWidth());
      yoStepGroundClearance.set(defaultXGaitSettings.getStepGroundClearance());
      yoEndPhaseShift.set(defaultXGaitSettings.getEndPhaseShift());

      paceStepDuration.set(defaultXGaitSettings.getPaceStepDuration());
      crawlStepDuration.set(defaultXGaitSettings.getCrawlStepDuration());
      trotStepDuration.set(defaultXGaitSettings.getTrotStepDuration());

      paceDoubleSupportFraction.set(defaultXGaitSettings.getPaceDoubleSupportFraction());
      crawlDoubleSupportFraction.set(defaultXGaitSettings.getCrawlDoubleSupportFraction());
      trotDoubleSupportFraction.set(defaultXGaitSettings.getTrotDoubleSupportFraction());

      endDoubleSupportDuration.set(defaultXGaitSettings.getEndDoubleSupportDuration());

      useFractionalDoubleSupport.set(defaultXGaitSettings.useFractionalDoubleSupport());

      parentRegistry.addChild(registry);
   }

   @Override
   public double getStanceLength()
   {
      return yoStanceLength.getDoubleValue();
   }

   @Override
   public double getStanceWidth()
   {
      return yoStanceWidth.getDoubleValue();
   }

   @Override
   public double getStepGroundClearance()
   {
      return yoStepGroundClearance.getDoubleValue();
   }

   @Override
   public double getEndPhaseShift()
   {
      return yoEndPhaseShift.getDoubleValue();
   }

   @Override
   public double getPaceStepDuration()
   {
      return paceStepDuration.getDoubleValue();
   }

   @Override
   public double getTrotStepDuration()
   {
      return trotStepDuration.getDoubleValue();
   }

   @Override
   public double getCrawlStepDuration()
   {
      return crawlStepDuration.getDoubleValue();
   }

   @Override
   public double getPaceDoubleSupportFraction()
   {
      return paceDoubleSupportFraction.getDoubleValue();
   }

   @Override
   public double getCrawlDoubleSupportFraction()
   {
      return crawlDoubleSupportFraction.getDoubleValue();
   }

   @Override
   public double getTrotDoubleSupportFraction()
   {
      return trotDoubleSupportFraction.getDoubleValue();
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return endDoubleSupportDuration.getDoubleValue();
   }

   @Override
   public boolean useFractionalDoubleSupport()
   {
      return useFractionalDoubleSupport.getBooleanValue();
   }

   public void setStanceLength(double stanceLength)
   {
      yoStanceLength.set(stanceLength);
   }

   public void setStanceWidth(double stanceWidth)
   {
      yoStanceWidth.set(stanceWidth);
   }

   public void setStepGroundClearance(double stepGroundClearance)
   {
      yoStepGroundClearance.set(stepGroundClearance);
   }

   public void setPaceStepDuration(double stepDuration)
   {
      paceStepDuration.set(stepDuration);
   }

   public void setCrawlStepDuration(double stepDuration)
   {
      crawlStepDuration.set(stepDuration);
   }

   public void setTrotStepDuration(double stepDuration)
   {
      trotStepDuration.set(stepDuration);
   }

   public void setPaceDoubleSupportFraction(double doubleSupportFraction)
   {
      paceDoubleSupportFraction.set(doubleSupportFraction);
   }

   public void setCrawlDoubleSupportFraction(double doubleSupportFraction)
   {
      crawlDoubleSupportFraction.set(doubleSupportFraction);
   }

   public void setTrotDoubleSupportFraction(double doubleSupportFraction)
   {
      trotDoubleSupportFraction.set(doubleSupportFraction);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      yoEndPhaseShift.set(endPhaseShift);
   }

   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      this.endDoubleSupportDuration.set(endDoubleSupportDuration);
   }

   public void setUseFractionalDoubleSupport(boolean useFractionalDoubleSupport)
   {
      this.useFractionalDoubleSupport.set(useFractionalDoubleSupport);
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
