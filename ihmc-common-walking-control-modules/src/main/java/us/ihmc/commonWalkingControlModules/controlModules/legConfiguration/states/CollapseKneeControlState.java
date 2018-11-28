package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlToolbox;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CollapseKneeControlState implements LegControlState
{
   private final YoDouble desiredFractionOfMidRangeForCollapsed;

   private final LegConfigurationControlToolbox toolbox;

   private final DoubleProvider collapsingDuration;

   private double desiredKneePosition;

   public CollapseKneeControlState(String namePrefix, LegConfigurationControlToolbox toolbox, DoubleProvider collapsingDuration, YoVariableRegistry registry)
   {
      this.toolbox = toolbox;
      this.collapsingDuration = collapsingDuration;

      desiredFractionOfMidRangeForCollapsed = new YoDouble(namePrefix + "DesiredFractionOfMidRangeForCollapsed", registry);
      desiredFractionOfMidRangeForCollapsed.set(toolbox.getParameters().getDesiredFractionOfMidrangeForCollapsedAngle());

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return false;
   }

   @Override
   public void doAction(double timeInState)
   {
      double collapsedAngle = desiredFractionOfMidRangeForCollapsed.getDoubleValue() * toolbox.getKneeMidRangeOfMotion();
      //         double alpha = MathTools.clamp(timeInState / collapsingDuration.getDoubleValue(), 0.0, 1.0);
      double alpha = computeQuadraticCollapseFactor(timeInState, collapsingDuration.getValue());
      desiredKneePosition = InterpolationTools.linearInterpolate(toolbox.getDesiredStraightLegAngle().getDoubleValue(), collapsedAngle, alpha);
//      desiredKneePosition = Math.max(desiredKneePosition, desiredAngleWhenStraight.getDoubleValue());
   }

   private double computeConstantCollapseFactor(double timeInState, double duration)
   {
      return Math.max(timeInState / duration, 0.0);
   }

   private double computeQuadraticCollapseFactor(double timeInState, double duration)
   {
      return MathTools.clamp(Math.pow(timeInState / duration, 2.0), 0.0, 1.0);
   }

   @Override
   public void onEntry()
   {
      toolbox.setLegControlWeight(LegConfigurationControlModule.LegControlWeight.MEDIUM);
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public LegConfigurationGainsReadOnly getLegConfigurationGains()
   {
      return toolbox.getStraightLegGains();
   }

   @Override
   public double getKneePitchPrivilegedConfiguration()
   {
      return desiredKneePosition;
   }
}
