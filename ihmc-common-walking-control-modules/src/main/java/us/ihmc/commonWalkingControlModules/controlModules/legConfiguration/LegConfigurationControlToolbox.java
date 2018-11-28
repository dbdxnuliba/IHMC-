package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule.LegControlWeight;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.YoLegConfigurationGains;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class LegConfigurationControlToolbox
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoEnum<LegControlWeight> legControlWeight;

   private final YoDouble desiredStraightLegAngle;

   private final YoBoolean useBracingAngle;


   private final LegConfigurationGainsReadOnly straightLegGains;
   private final LegConfigurationGainsReadOnly bentLegGains;

   private final double kneeSquareRangeOfMotion;
   private final double kneeRangeOfMotion;
   private final double kneeMidRangeOfMotion;

   private final OneDoFJointBasics kneePitchJoint;

   private final LegConfigurationParameters parameters;

   public LegConfigurationControlToolbox(String sidePrefix, OneDoFJointBasics kneePitchJoint, LegConfigurationParameters parameters, YoVariableRegistry parentRegistry)
   {
      String namePrefix = sidePrefix + "Leg";
      this.kneePitchJoint = kneePitchJoint;
      this.straightLegGains = new YoLegConfigurationGains("straight", parameters.getStraightLegGains(), registry);
      this.bentLegGains = new YoLegConfigurationGains("bent", parameters.getBentLegGains(), registry);
      this.parameters = parameters;

      legControlWeight = YoEnum.create(namePrefix + "LegControlWeight", "", LegControlWeight.class, registry, false);

      desiredStraightLegAngle = new YoDouble(namePrefix + "DesiredStraightLegAngle", registry);

      useBracingAngle = new YoBoolean(namePrefix + "UseBracingAngle", registry);

      double kneeLimitUpper = kneePitchJoint.getJointLimitUpper();
      if (Double.isNaN(kneeLimitUpper) || Double.isInfinite(kneeLimitUpper))
         kneeLimitUpper = Math.PI;
      double kneeLimitLower = kneePitchJoint.getJointLimitLower();
      if (Double.isNaN(kneeLimitLower) || Double.isInfinite(kneeLimitLower))
         kneeLimitLower = -Math.PI;
      kneeSquareRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
      kneeRangeOfMotion = kneeLimitUpper - kneeLimitLower;
      kneeMidRangeOfMotion = 0.5 * (kneeLimitUpper + kneeLimitLower);

      parentRegistry.addChild(registry);
   }

   public LegConfigurationGainsReadOnly getStraightLegGains()
   {
      return straightLegGains;
   }

   public LegConfigurationGainsReadOnly getBentLegGains()
   {
      return bentLegGains;
   }

   public YoDouble getDesiredStraightLegAngle()
   {
      return desiredStraightLegAngle;
   }

   public YoEnum<LegControlWeight> getLegControlWeight()
   {
      return legControlWeight;
   }

   public void setLegControlWeight(LegControlWeight legControlWeight)
   {
      this.legControlWeight.set(legControlWeight);
   }

   public double getKneeMidRangeOfMotion()
   {
      return kneeMidRangeOfMotion;
   }

   public double getKneeSquareRangeOfMotion()
   {
      return kneeSquareRangeOfMotion;
   }

   public double getKneeRangeOfMotion()
   {
      return kneeRangeOfMotion;
   }

   public OneDoFJointBasics getKneePitchJoint()
   {
      return kneePitchJoint;
   }

   public LegConfigurationParameters getParameters()
   {
      return parameters;
   }

   public void setUseBracingAngle(boolean useBracingAngle)
   {
      this.useBracingAngle.set(useBracingAngle);
   }

   public boolean useBracingAngle()
   {
      return useBracingAngle.getBooleanValue();
   }
}
