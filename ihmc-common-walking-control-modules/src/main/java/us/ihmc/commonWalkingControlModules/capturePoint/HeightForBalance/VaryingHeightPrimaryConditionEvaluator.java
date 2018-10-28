package us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance;

import us.ihmc.commons.MathTools;

public class VaryingHeightPrimaryConditionEvaluator
{
   private VaryingHeightPrimaryConditionEnum primaryConditionEnum;
   private VaryingHeightPrimaryConditionEnum primaryConditionPreviousTick;
   private final double zMin;
   private final double minKneeAngle;
   private final double maxKneeAngle;
   private final double aMinPredicted;
   private final double aMaxPredicted;

   private boolean primaryConditionHasChanged;

   public VaryingHeightPrimaryConditionEvaluator(double zMin, double minKneeAngle, double maxKneeAngle, double aMinPredicted, double aMaxPredicted)
   {
     this.zMin=zMin;
     this.minKneeAngle=minKneeAngle;
     this.maxKneeAngle=maxKneeAngle;
     this.aMinPredicted=aMinPredicted;
     this.aMaxPredicted=aMaxPredicted;
     primaryConditionEnum = VaryingHeightPrimaryConditionEnum.DEFAULT;
     primaryConditionPreviousTick = primaryConditionEnum;
   }

   public VaryingHeightPrimaryConditionEnum computeAndGetPrimaryConditionEnum(double z, double dz, double zMax, double kneeAngle, double errorAngle, double errorAngleEndOfSwing,
                                                                              boolean angleGrows, double negAlignTresh, double posAlignTresh,
                                                                              VaryingHeightPrimaryConditionEnum varyingHeightConditionPreviousTick, boolean useAngleForConditions,
                                                                              boolean distancePosAlignment, double copCoMOrthDistance)
   {
      this.primaryConditionPreviousTick = varyingHeightConditionPreviousTick;
      if((z+MathTools.sign(dz)*dz*dz/(2* -aMinPredicted)>zMax|| kneeAngle<minKneeAngle)    )                             // max height/vel and singularity
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.MAXZ;
      }
      else if (z+MathTools.sign(dz)*dz*dz/(2*aMaxPredicted)<zMin || kneeAngle >maxKneeAngle )                         // min height / min vel
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.MINZ;
      }
      else if(errorAngle<posAlignTresh && errorAngle>-posAlignTresh ) //||(!useAngleForConditions && distancePosAlignment)                                         // alignment ICPe and 'pendulum' positive force
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.ALIGNED_POS;
         if(varyingHeightConditionPreviousTick==VaryingHeightPrimaryConditionEnum.PREPARE_POS && angleGrows && MathTools.epsilonEquals(errorAngle,posAlignTresh,0.2))
         {
            primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_POS;
         }
         else if (varyingHeightConditionPreviousTick==VaryingHeightPrimaryConditionEnum.PREPARE_POS && !angleGrows && MathTools.epsilonEquals(errorAngle,-posAlignTresh,0.2))
         {
            primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_POS;
         }
      }
      else if(errorAngle>negAlignTresh|| errorAngle<-negAlignTresh  )   //||(!useAngleForConditions && !distancePosAlignment)                                                         // alignment ICPe and 'pendulum' negative force
      {
         primaryConditionEnum = VaryingHeightPrimaryConditionEnum.ALIGNED_NEG;
         if(varyingHeightConditionPreviousTick==VaryingHeightPrimaryConditionEnum.PREPARE_NEG && angleGrows && MathTools.epsilonEquals(errorAngle,-negAlignTresh,0.2))
         {
            primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_NEG;
         }
         else if (varyingHeightConditionPreviousTick==VaryingHeightPrimaryConditionEnum.PREPARE_NEG && !angleGrows && MathTools.epsilonEquals(errorAngle,negAlignTresh,0.2))
         {
            primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_NEG;
         }
      }
      else //if (useAngleForConditions)      // preparing for future angle
      {
         if(errorAngleEndOfSwing<posAlignTresh && errorAngleEndOfSwing>-posAlignTresh)
         {
            primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_NEG;
            if(varyingHeightConditionPreviousTick==VaryingHeightPrimaryConditionEnum.ALIGNED_POS)
            {
               primaryConditionEnum = VaryingHeightPrimaryConditionEnum.ALIGNED_POS;
            }
         }
         else if (errorAngleEndOfSwing>negAlignTresh || errorAngleEndOfSwing<-negAlignTresh)
         {
            primaryConditionEnum = VaryingHeightPrimaryConditionEnum.PREPARE_POS;
            if(varyingHeightConditionPreviousTick==VaryingHeightPrimaryConditionEnum.ALIGNED_NEG)
            {
               primaryConditionEnum = VaryingHeightPrimaryConditionEnum.ALIGNED_NEG;
            }
         }
      }
      return primaryConditionEnum;
   }
   public boolean getPrimaryConditionHasChanged()
   {
      if(primaryConditionPreviousTick!=primaryConditionEnum)
      {
         primaryConditionHasChanged = true;
      }
      else
      {
         primaryConditionHasChanged = false;
      }
      return primaryConditionHasChanged;
   }
}
