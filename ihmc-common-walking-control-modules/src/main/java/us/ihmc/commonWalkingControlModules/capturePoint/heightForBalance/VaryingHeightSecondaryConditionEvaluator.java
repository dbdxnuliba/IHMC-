package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import us.ihmc.commons.MathTools;

import static java.lang.Math.asin;
import static java.lang.Math.sqrt;

public class VaryingHeightSecondaryConditionEvaluator
{
   private final double zMin;
   private double aMinCtrl;
   private double aMaxCtrl;
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

   public VaryingHeightSecondaryConditionEvaluator(double zMin, double tHalfWaySwing, double epsilonForSmooth,
                                                   VaryingHeightTimeToConstraintsPredictor timeToConstraintsPredictor)
   {
      this.zMin = zMin;
      this.tHalfWaySwing = tHalfWaySwing;
      this.epsilonForSmooth =epsilonForSmooth;
      this.timeToConstraintsPredictor=timeToConstraintsPredictor;
      aCtrl=0.0;
      tToConst=0.0;
      secondaryCondition = VaryingHeightSecondaryConditionEnum.DEFAULT;
      secondaryConditionPreviousTick = secondaryCondition;
   }
   /**
    * Evaluate secondary condition.
    */
   public VaryingHeightSecondaryConditionEnum computeAndGetSecondaryConditionEnum(double aMinCtrl, double aMaxCtrl,double z, double dz,VaryingHeightPrimaryConditionEnum primaryCondition, boolean primaryConditionHasChanged, VaryingHeightSecondaryConditionEnum secondaryConditionPreviousTick,
                                                                                  double tInState, double tToMinVelocityPredicted, double tToMinPositionPredicted,
                                                                                  double tToMaxVelocityPredicted, double tToMaxPositionPredicted, double tRemainingEndOfWalkingState, double errorAngle,
                                                                                  double errorAngleEndOfSwing, boolean angleGrows, double posAlignTresh, double negAlignTresh, double zMax, boolean nonDynamicCase)
   {
      this.secondaryConditionPreviousTick=secondaryConditionPreviousTick;
      this.modifiedPosAlignTresh=posAlignTresh;
      this.aMinCtrl=aMinCtrl;
      this.aMaxCtrl=aMaxCtrl;

      // see below
      computeTToSwitch(primaryCondition,angleGrows,posAlignTresh,negAlignTresh,errorAngle,errorAngleEndOfSwing,tRemainingEndOfWalkingState,zMax, 0.6*aMinCtrl,0.6*aMaxCtrl);
      // see below
      computeTToConstraint(primaryCondition,tToMinVelocityPredicted,tToMinPositionPredicted,tToMaxVelocityPredicted,tToMaxPositionPredicted);


      /**
       * With Prepare, or if time to constraint is very close to time to switch: SMOOTH.
       */
      if (((tInState > tHalfWaySwing && MathTools.epsilonEquals(tToConst, tToSwitch, epsilonForSmooth))
            || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_POS || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_NEG)
            && tToConst < tToSwitch)

      {
         secondaryCondition = VaryingHeightSecondaryConditionEnum.SMOOTH;
      }
      /**
       * If time to constraint is smaller than time to switch: HOLD. From SMOOTH into HOLD again not possible.
       */
      else if (tInState > tHalfWaySwing && tToConst < tToSwitch || (nonDynamicCase && (primaryCondition==VaryingHeightPrimaryConditionEnum.MAXZ && dz<0.0 || primaryCondition==VaryingHeightPrimaryConditionEnum.MINZ && dz>0.0)))
      {
         secondaryCondition = VaryingHeightSecondaryConditionEnum.HOLD;
         if (secondaryConditionPreviousTick == VaryingHeightSecondaryConditionEnum.SMOOTH)
         {
            secondaryCondition = VaryingHeightSecondaryConditionEnum.SMOOTH;
         }
      }
      /**
       * Default case (min/max control input). In the same condition, cannot be after HOLD or SMOOTH.
       */
      else
      {
         secondaryCondition = VaryingHeightSecondaryConditionEnum.DEFAULT;
         if (!primaryConditionHasChanged && secondaryConditionPreviousTick == VaryingHeightSecondaryConditionEnum.HOLD)
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

   /**
    * Computes time to the moment when threshold is met. Just linear proportional to the relative error angle to that point, error angle end of swing and time
    * remaining to end of swing.
    */
   private void computeTToSwitch(VaryingHeightPrimaryConditionEnum primaryCondition, boolean angleGrows, double posAlignTresh, double negAlignTresh,
                                 double errorAngle, double errorAngleEndOfSwing, double tRemainingEndOfWalkingState, double zMax, double aMinPredicted,
                                 double aMaxPredicted)
   {
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
         // see below
         modifyPosAlignTreshold(tRemainingEndOfWalkingState,posAlignTresh,zMax, aMinPredicted,aMaxPredicted);
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
      else
      {
         tToSwitch=tRemainingEndOfWalkingState;
      }
   }

   /**
    * Determines which predicted constraint time is closest and suited for the scenario of all predicted constraint times. Also calculates with that aSmooth which linearly scales
    * the max control law (would correspond to a velocity constraint to be logical. tried below for position too, but couldn't get it stable yet)
    */
   private void computeTToConstraint(VaryingHeightPrimaryConditionEnum primaryCondition, double tToMinVelocityPredicted, double tToMinPositionPredicted,
                                     double tToMaxVelocityPredicted, double tToMaxPositionPredicted)
   {
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
         aSmooth = MathTools.clamp(aSmooth,aMinCtrl,0);
      }
      if (primaryCondition == VaryingHeightPrimaryConditionEnum.ALIGNED_POS || primaryCondition == VaryingHeightPrimaryConditionEnum.MINZ
            || primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_POS)
      {
         aCtrl = aMaxCtrl;
         if(tToMaxVelocityPredicted<tToMaxPositionPredicted)
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
         aSmooth=MathTools.clamp(aSmooth,0,aMaxCtrl);
      }
   }

   /**
    * Threshold for pos alignment is modified when the time after the predicted threshold is higher than the time to the max position at that moment.
    * @param tRemainingEndOfWalkingState
    * @param posAlignTresh
    * @param zMax
    * @param aMinPredicted
    * @param aMaxPredcited
    */
   private void modifyPosAlignTreshold(double tRemainingEndOfWalkingState, double posAlignTresh, double zMax, double aMinPredicted, double aMaxPredcited)
   {
      if (tRemainingEndOfWalkingState -tToSwitch > timeToConstraintsPredictor.getTMaxPosReachedPredicted(zMin, 0,zMax,aMinPredicted,aMaxPredcited))
      {
         modifiedPosAlignTresh = timeToConstraintsPredictor.getTMaxPosReachedPredicted(zMin, 0,zMax,aMinPredicted,aMaxPredcited) / (tRemainingEndOfWalkingState-tToSwitch) * posAlignTresh;
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
