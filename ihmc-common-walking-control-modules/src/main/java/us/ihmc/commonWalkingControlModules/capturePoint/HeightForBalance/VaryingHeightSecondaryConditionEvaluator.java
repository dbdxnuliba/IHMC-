package us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance;

import us.ihmc.commons.MathTools;

import static java.lang.Math.sqrt;

public class VaryingHeightSecondaryConditionEvaluator
{
   private final double zMin;
   private final double aMinCtrl;
   private final double aMaxCtrl;
   private double aCtrl;
   private double aSmooth;
   private double tToConst;
   private double tToSwitch;
   private double tHalfWaySwing;
   private double epsilonForSmooth;
   private double modifiedPosAlignTresh;

   private VaryingHeightSecondaryConditionEnum secondaryCondition;
   private VaryingHeightSecondaryConditionEnum secondaryConditionPreviousTick;
   private VaryingHeightTimeToConstraintsPredictor timeToConstraintsPredictor;

   public VaryingHeightSecondaryConditionEvaluator(double zMin, double aMinCtrl, double aMaxCtrl, double tHalfWaySwing, double epsilonForSmooth,
                                                   VaryingHeightTimeToConstraintsPredictor timeToConstraintsPredictor)
   {
      this.zMin = zMin;
      this.aMinCtrl=aMinCtrl;
      this.aMaxCtrl=aMaxCtrl;
      this.tHalfWaySwing = tHalfWaySwing;
      this.epsilonForSmooth =epsilonForSmooth;
      this.timeToConstraintsPredictor=timeToConstraintsPredictor;
      aCtrl=0.0;
      tToConst=0.0;
      secondaryCondition = VaryingHeightSecondaryConditionEnum.DEFAULT;
      secondaryConditionPreviousTick = secondaryCondition;
   }

   public VaryingHeightSecondaryConditionEnum computeAndGetSecondaryConditionEnum(double z, double dz,VaryingHeightPrimaryConditionEnum primaryCondition, boolean primaryConditionHasChanged, VaryingHeightSecondaryConditionEnum secondaryConditionPreviousTick,
                                                                                  double tInState, double tToMinVelocityPredicted, double tToMinPositionPredicted,
                                                                                  double tToMaxVelocityPredicted, double tToMaxPositionPredicted, double tRemainingEndOfWalkingState, double errorAngle,
                                                                                  double errorAngleEndOfSwing, boolean angleGrows, double posAlignTresh, double negAlignTresh, double zMax)
   {
      this.secondaryConditionPreviousTick=secondaryConditionPreviousTick;
      tToSwitch=tRemainingEndOfWalkingState;
      this.modifiedPosAlignTresh=posAlignTresh;


      if (primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_NEG)
      {
         if (angleGrows)
         {
            tToSwitch = (-posAlignTresh - errorAngle) / (errorAngleEndOfSwing - errorAngle) * tRemainingEndOfWalkingState;
         }
         else
         {
            tToSwitch = (posAlignTresh - errorAngle) / (errorAngleEndOfSwing - errorAngle) * tRemainingEndOfWalkingState;
         }
         modifyPosAlignTreshold(tRemainingEndOfWalkingState,posAlignTresh,zMax);
      }
      else if (primaryCondition== VaryingHeightPrimaryConditionEnum.PREPARE_POS)
      {
         if (angleGrows)
         {
            tToSwitch = (-negAlignTresh - errorAngle) / (errorAngleEndOfSwing - errorAngle) * tRemainingEndOfWalkingState;
         }
         else
         {
            tToSwitch = (negAlignTresh - errorAngle) / (errorAngleEndOfSwing - errorAngle) * tRemainingEndOfWalkingState;
         }
      }

      if (primaryCondition== VaryingHeightPrimaryConditionEnum.ALIGNED_NEG || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_NEG
            || primaryCondition == VaryingHeightPrimaryConditionEnum.MAXZ)
      {
         aCtrl = aMinCtrl;
         if(tToMinVelocityPredicted<tToMinPositionPredicted)
         {
            tToConst = tToMinVelocityPredicted;
            aSmooth = tToMinVelocityPredicted*aCtrl/tToSwitch;
         }
         else
         {
            tToConst = tToMinPositionPredicted;
            aSmooth = tToMinPositionPredicted*aCtrl/tToSwitch;
            /*
            aSmooth = tToMinPositionPredicted*tToMinPositionPredicted*aCtrl/tToSwitch/tToSwitch;
            aSmooth = (-0.6-dz)/tToSwitch;
            double a = -0.5*tToConst*tToConst/5;
            double b = 0.5*tToConst*tToConst - dz*tToConst/5;
            double c = 0.5*dz*dz/5+dz*tToConst+z-zMin;
            aSmooth = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            */

         }
      }
      if (primaryCondition == VaryingHeightPrimaryConditionEnum.ALIGNED_POS || primaryCondition == VaryingHeightPrimaryConditionEnum.MINZ
            || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_POS)
      {
         aCtrl = aMaxCtrl;
         if(tToMinVelocityPredicted<tToMinPositionPredicted)
         {
            tToConst = tToMaxVelocityPredicted;
            aSmooth = tToMaxVelocityPredicted*aCtrl/tToSwitch;
         }
         else
         {
            tToConst = tToMaxPositionPredicted;
            aSmooth = tToMaxPositionPredicted*aCtrl/tToSwitch;
            /*
            aSmooth = tToMaxPositionPredicted*tToMaxPositionPredicted*aCtrl/tToSwitch/tToSwitch;
            aSmooth = (0.7-dz)/tToSwitch;
            double a = 0.5*tToConst*tToConst/5;
            double b = 0.5*tToConst*tToConst + dz*tToConst/5;
            double c = 0.5*dz*dz/5+dz*tToConst+z-zMax;
            aSmooth = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            */
         }
      }

      /**
       * Evaluate secondary conditions
       */
      if (((tInState > tHalfWaySwing && MathTools.epsilonEquals(tToConst, tToSwitch, epsilonForSmooth))
            || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_POS || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_NEG)
            && tToConst < tToSwitch)

      {
         secondaryCondition = VaryingHeightSecondaryConditionEnum.SMOOTH;
      }
      else if (tInState > tHalfWaySwing && tToConst < tToSwitch)
      {
         secondaryCondition = VaryingHeightSecondaryConditionEnum.HOLD;
         if (secondaryConditionPreviousTick == VaryingHeightSecondaryConditionEnum.SMOOTH)
         {
            secondaryCondition = VaryingHeightSecondaryConditionEnum.SMOOTH;
         }
      }
      else
      {
         secondaryCondition = VaryingHeightSecondaryConditionEnum.DEFAULT;
         if (secondaryConditionPreviousTick == VaryingHeightSecondaryConditionEnum.HOLD)
         {
            secondaryCondition = VaryingHeightSecondaryConditionEnum.HOLD;
         }
         else if (!primaryConditionHasChanged && secondaryConditionPreviousTick == VaryingHeightSecondaryConditionEnum.SMOOTH)
         {
            secondaryCondition = VaryingHeightSecondaryConditionEnum.SMOOTH;
         }
      }
      return secondaryCondition;
   }

   private void modifyPosAlignTreshold(double tRemainingEndOfWalkingState, double posAlignTresh, double zMax)
   {
      if (tRemainingEndOfWalkingState -tToSwitch > timeToConstraintsPredictor.getTMaxPosReachedPredicted(zMin, 0,zMax))
      {
         modifiedPosAlignTresh = timeToConstraintsPredictor.getTMaxPosReachedPredicted(zMin, 0,zMax) / (tRemainingEndOfWalkingState-tToSwitch) * posAlignTresh;
      }
      else
      {
         modifiedPosAlignTresh = posAlignTresh;
      }
   }

   public double getControlSmooth()
   {
      return aSmooth;
   }

   public double getControlBound()
   {
      return aCtrl;
   }

   public double getTimeToClosestConstraint()
   {
      return tToConst;
   }

   public double getTimeToConditionSwitch()
   {
      return  tToSwitch;
   }

   public double getModifiedPosAlignTresh()
   {
      return modifiedPosAlignTresh;
   }
}
