package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import us.ihmc.commons.MathTools;

public class VaryingHeightPrimaryConditionEvaluator
{
   private VaryingHeightPrimaryConditionEnum primaryConditionEnum;
   private final double zMin;

   private boolean primaryConditionHasChanged;

   public VaryingHeightPrimaryConditionEvaluator(double zMin)
   {
      this.zMin = zMin;

      primaryConditionEnum = VaryingHeightPrimaryConditionEnum.DEFAULT;
   }

   public VaryingHeightPrimaryConditionEnum computeAndGetPrimaryConditionEnum(double errorAngle, double errorAngleEndOfSwing, double negAlignTresh,
                                                                              double posAlignTresh, boolean useAngleForConditions, boolean distancePosAlignment,
                                                                              double copCoMProjDistance, double copCoMProjDistanceEndOfSwing,
                                                                              boolean nonDynamicCase, double copCoMProjMinDistance)
   {

      if ((useAngleForConditions && (errorAngle < posAlignTresh && errorAngle > -posAlignTresh) || (!useAngleForConditions && distancePosAlignment))
            && copCoMProjDistance > copCoMProjMinDistance)
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.ALIGNED_POS;
      }
      else if (useAngleForConditions && copCoMProjDistanceEndOfSwing > copCoMProjMinDistance && (errorAngleEndOfSwing < posAlignTresh
            && errorAngleEndOfSwing > -posAlignTresh))
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_NEG;
      }
      else
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.DEFAULT;
      }
      return primaryConditionEnum;
   }
}