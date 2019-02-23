package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.awt.*;

import static java.lang.Math.*;

public class VaryingHeightControlModule implements VaryingHeightControlModuleInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("HeightForBalance");
   private WalkingControllerParameters walkingControllerParameters;

   private FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private FramePoint2D desiredCMPtoProject = new FramePoint2D();
   private FrameVector2D icpError = new FrameVector2D();
   private FramePoint2D com2DtoProject = new FramePoint2D();
   private FramePoint2D com2DtoProjectEndOfSwing = new FramePoint2D();
   private FramePoint3D com3D = new FramePoint3D();
   private FrameVector3DReadOnly comVelocity3D = new FrameVector3D();
   private FrameVector3D linearMomentumRateOfChangeFromLIP = new FrameVector3D();
   private YoFramePoint2D yoCoMEndOfSwing2DNotHacky;

   private FrameVector2D desiredICPVelocity = new FrameVector2D();

   private YoFramePoint2D yoProjectedDesiredCMP;
   private YoFramePoint2D yoProjectedCoM2D;
   private final YoEnum<VaryingHeightPrimaryConditionEnum> primaryConditionYoEnum;
   private boolean calculateUseAngleForConditions;

   private double totalMass;
   private double stateClock;
   private YoDouble yoTimeInState;

   private double desiredHeightAcceleration = 0;
   private double desiredHeightAccelerationPreviousTick = 0;
   private YoDouble yoDesiredHeightAcceleration;

   private RobotSide supportSide = null;

   private FrameVector2D addedHorizontalAcceleration = new FrameVector2D();

   private FrameVector3D modifiedLinearMomentumRateOfChange = new FrameVector3D();

   private HighLevelHumanoidControllerToolbox controllerToolbox;

   private OneDoFJointBasics kneeJoint;

   private double kneeAngle;

   private boolean isInDoubleSupport;

   private YoDouble copCoMICPeAngle;
   private YoDouble copCoMICPeAngleFinal;

   private boolean useAngleForConditions;
   private YoBoolean yoUseAngleForConditions;

   private YoDouble yoTimeMinPosReached;
   private YoDouble yoTimeMaxPosReached;
   private YoDouble yoTimeRemaining;
   private YoDouble yoTimeToSwitch;

   private double posAlignTresh;
   private double negAlignTresh;
   private double aMaxCtrl;
   private double aMinCtrl;
   private double aMaxCtrlAngle;
   private double aMinCtrlAngle;


   private double jMax;
   private double dt;
   private double aMaxPredicted;
   private double aMinPredicted;
   private double fracOfAForPrediction;
   private double zMax;
   private double zMaxStartSwing;
   private double zMaxTouchDown;
   private double zMin;
   private double tf;
   private double tForHalfWaySwing;
   private double tRemainingEndOfWalkingState;
   private boolean nonDynamicCase;
   private double copCoMProjMinDistance;
   private double minICPerror;

   boolean walkingStateSwitch;
   private YoBoolean yoWalkingStateSwitch;

   private YoDouble yoCoMHeightVelocity;
   private YoDouble yozMax;
   private YoDouble yozMaxAlpha;

   boolean heightControlInThisWalkingState = false;
   boolean dsTrajectoryIsGenerated = false;

   VaryingHeightAngleAndDistanceEvaluator angleAndDistanceEvaluator;
   VaryingHeightPrimaryConditionEvaluator primaryConditionEvaluator;
   VaryingHeightTimeToConstraintsPredictor timeToConstraintsPredictor;

   private YoDouble yoZMaxStart;
   private YoDouble yoZMaxTouchDown;
   private YoDouble yoZMin;
   private YoDouble yoAMaxControlAngle;
   private YoDouble yoAMinControlAngle;
   private YoDouble yoAMaxControlDistance;
   private YoDouble yoAMinControlDistance;
   private YoDouble yoJMax;
   private YoDouble yoFracAPred;
   private YoDouble yoCoPProjectedCoMMinDistance;
   private YoDouble yoMinICPError;

   boolean firstDone;
   boolean secondDone;

   double z;
   double dz;
   VaryingHeightPrimaryConditionEnum primaryCondition;

   public VaryingHeightControlModule(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry,
                                     YoGraphicsListRegistry yoGraphicsListRegistry, WalkingControllerParameters walkingControllerParameters)
   {
      this.controllerToolbox = controllerToolbox;
      this.totalMass = totalMass;
      this.walkingControllerParameters = walkingControllerParameters;
      parentRegistry.addChild(registry);
      dt = controllerToolbox.getControlDT();
      yoZMaxStart = new YoDouble(getClass().getSimpleName()+"MaxHeightStartSwing",registry);
      yoZMaxTouchDown = new YoDouble(getClass().getSimpleName()+"MaxHeightTouchDown",registry);
      yoZMin = new YoDouble(getClass().getSimpleName()+"MinHeight",registry);
      yoAMaxControlAngle = new YoDouble(getClass().getSimpleName()+"AMaxForAngleCase",registry);
      yoAMinControlAngle = new YoDouble(getClass().getSimpleName()+"AMinForAngleCase",registry);
      yoAMaxControlDistance = new YoDouble(getClass().getSimpleName()+"AMaxControlDistanceCase",registry);
      yoAMinControlDistance = new YoDouble(getClass().getSimpleName()+"AMinControlDistanceCase",registry);
      yoJMax = new YoDouble(getClass().getSimpleName()+"MaxJerk",registry);
      yoFracAPred = new YoDouble(getClass().getSimpleName()+"FractionOfAMaxUsedInPrediction",registry);
      yoCoPProjectedCoMMinDistance = new YoDouble(getClass().getSimpleName()+"CoPtoProjectedCoMOnICPeDistance",registry);
      yoMinICPError = new YoDouble(getClass().getSimpleName()+"MinICPError",registry);

      yoDesiredHeightAcceleration = new YoDouble("DesiredHeightAccelerationHeightControl", registry);

      // error angle, error angle end of swing and error angle direction
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      copCoMICPeAngleFinal = new YoDouble("CoPCoMICPeAngleFinal", registry);

      yoCoMEndOfSwing2DNotHacky = new YoFramePoint2D("varyingHeightCoMEndOfSwingFromPlanner", ReferenceFrame.getWorldFrame(), registry);
      yoCoMHeightVelocity = new YoDouble("heightForBalanceVelocity", registry);

      // angle and distance booleans
      yoUseAngleForConditions = new YoBoolean("useAngleForConditions", registry);

      yoWalkingStateSwitch = new YoBoolean("stateSwitch", registry);

      // estimated times
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);
      yoTimeToSwitch = new YoDouble("estTimeToSwitch", registry);

      // threshold for positive alignment, can be modifed
      yozMax = new YoDouble("zMaxHeightCtrl",registry);
      yozMaxAlpha = new YoDouble("zMaxAlphaHeightCtrl",registry);

      yoTimeToSwitch.set(walkingControllerParameters.getDefaultSwingTime());

      primaryConditionYoEnum = new YoEnum<>("varyingHeightCondition", registry, VaryingHeightPrimaryConditionEnum.class);

      yoTimeInState = new YoDouble("varyingHeightTimeInState", registry);

      String label = getClass().getSimpleName();
      ArtifactList artifacts = new ArtifactList(label);

      yoProjectedDesiredCMP = new YoFramePoint2D(label + "projCMPd", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CMPd", yoProjectedDesiredCMP.getYoX(), yoProjectedDesiredCMP.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED,
                                           0.025));

      yoProjectedCoM2D = new YoFramePoint2D(label + "projCOM2D", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CoM2D", yoProjectedCoM2D.getYoX(), yoProjectedCoM2D.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.01));

      primaryConditionYoEnum.set(VaryingHeightPrimaryConditionEnum.DEFAULT);
      artifacts.setVisible(true);
      yoGraphicsListRegistry.registerArtifactList(artifacts);

      // Parameters:
      yoZMaxStart.set(walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightFirstPhaseOfSwing());
      yoZMaxTouchDown.set(yoZMaxStart.getDoubleValue() * walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightFractionForSecondPhaseOfSwing());
      yoZMin.set(walkingControllerParameters.getHeightForBalanceParameters().getMinHeight());
      yoAMaxControlAngle.set(walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightAccelerationForAngleCase());
      yoAMinControlAngle.set(walkingControllerParameters.getHeightForBalanceParameters().getMinHeightAccelerationForAngleCase());
      yoAMaxControlDistance.set(walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightAccelerationForDistanceCase());
      yoAMinControlDistance.set(walkingControllerParameters.getHeightForBalanceParameters().getMinHeightAccelerationForDistanceCase());
      yoJMax.set(walkingControllerParameters.getHeightForBalanceParameters().getMaximumJerk());
      posAlignTresh = walkingControllerParameters.getHeightForBalanceParameters().getAnglePositiveAlignmentThresholdFromStart();
      negAlignTresh = walkingControllerParameters.getHeightForBalanceParameters().getAngleNegativeAlignmentThreshold();
      tForHalfWaySwing = walkingControllerParameters.getHeightForBalanceParameters().getFractionOfSwingTimeToChangeMaxHeight() * walkingControllerParameters
            .getDefaultSwingTime();
      yoFracAPred.set(walkingControllerParameters.getHeightForBalanceParameters().getFractionOfMaxHeightAccelerationToConsiderInPrediction());
      yoCoPProjectedCoMMinDistance.set(walkingControllerParameters.getHeightForBalanceParameters().getMinimumCoPCoMProjectedICPeDistanceToControl());
      yoMinICPError.set(0.03);

      angleAndDistanceEvaluator = new VaryingHeightAngleAndDistanceEvaluator();
      primaryConditionEvaluator = new VaryingHeightPrimaryConditionEvaluator(yoZMin.getDoubleValue());
      timeToConstraintsPredictor = new VaryingHeightTimeToConstraintsPredictor(yoZMin.getDoubleValue());
      primaryCondition = VaryingHeightPrimaryConditionEnum.DEFAULT;
   }

   public void compute()
   {
      zMaxStartSwing = yoZMaxStart.getDoubleValue();
      zMaxTouchDown = yoZMaxTouchDown.getDoubleValue();
      zMin = yoZMin.getDoubleValue();
      aMaxCtrlAngle = yoAMaxControlAngle.getDoubleValue();
      aMinCtrlAngle = yoAMinControlAngle.getDoubleValue();
      jMax = yoJMax.getDoubleValue();
      fracOfAForPrediction = yoFracAPred.getDoubleValue();
      copCoMProjMinDistance = yoCoPProjectedCoMMinDistance.getDoubleValue();
      minICPerror=yoMinICPError.getDoubleValue();


      stateClock = stateClock + dt;
      tf = walkingControllerParameters.getDefaultSwingTime();
      tRemainingEndOfWalkingState = tf - stateClock;
      yoTimeRemaining.set(tRemainingEndOfWalkingState);
      yoTimeInState.set(stateClock);

      desiredHeightAccelerationPreviousTick = desiredHeightAcceleration;

      kneeAngle = getKneeAngle();

      com2DtoProject.changeFrame(ReferenceFrame.getWorldFrame());
      com2DtoProjectEndOfSwing.changeFrame(ReferenceFrame.getWorldFrame());
      com3D.changeFrame(ReferenceFrame.getWorldFrame());
      desiredCMPtoProject.changeFrame(ReferenceFrame.getWorldFrame());
      icpError.changeFrame(ReferenceFrame.getWorldFrame());

      if(!supportPolygon.isPointInside(desiredCMPtoProject))
      {
         supportPolygon.orthogonalProjection(desiredCMPtoProject);
      }

      // For standing push
      nonDynamicCase = yoCoMEndOfSwing2DNotHacky.distanceFromOrigin()<0.5;

      // Reset values if state switch occurs, or if standing
      FrameVector2D comVelocity2D = new FrameVector2D();
      comVelocity2D.set(comVelocity3D);
      if (walkingStateSwitch
            || nonDynamicCase && icpError.length()<0.02 && comVelocity2D.length()<0.01)
      {
         heightControlInThisWalkingState = false;
         dsTrajectoryIsGenerated = false;
      }

      // Determines if height control or not
      boolean heightControlCondition = (
            icpError.length() > minICPerror && (isInDoubleSupport == false || nonDynamicCase == true) || heightControlInThisWalkingState == true);

      comVelocity3D = new FrameVector3D();
      comVelocity3D = controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity();
      z = com3D.getZ();
      dz = comVelocity3D.getZ();
      yoCoMHeightVelocity.set(dz);
      FrameVector2D centerOfMassVelocity2D = new FrameVector2D();
      centerOfMassVelocity2D.setIncludingFrame(comVelocity3D);
      centerOfMassVelocity2D.changeFrame(ReferenceFrame.getWorldFrame());

      // Determines if alignment angle should be used
      calculateUseAngleForConditions = (heightControlInThisWalkingState == false);


      // HEIGHT CONTROL
      if (heightControlCondition)
      {
         // different max heights
         zMax = getzMax(nonDynamicCase,stateClock,tForHalfWaySwing,useAngleForConditions);
         yozMax.set(zMax);
         // Min/max - control law set based on using angle or distance
         if (calculateUseAngleForConditions || walkingStateSwitch)
         {
            firstDone=false;
            secondDone=false;
            useAngleForConditions = angleAndDistanceEvaluator.getUseAngleForConditions(icpError);
            yoUseAngleForConditions.set(useAngleForConditions);
            // projected com positions
            FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
            desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
            desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProjectEndOfSwing);
            yoProjectedCoM2D.set(com2DtoProject);

            /**
             * Error angle current
             */
            double errorAngle = angleAndDistanceEvaluator.getErrorAngle(icpError, desiredCMPtoProject, com3D);
            copCoMICPeAngle.set(errorAngle);

            /**
             * Error angle end of swing
             */
            double errorAngleEndOfSwing = angleAndDistanceEvaluator.getErrorAngleEndOfSwing(yoCoMEndOfSwing2DNotHacky, desiredCMPtoProject, icpError);
            copCoMICPeAngleFinal.set(errorAngleEndOfSwing);
            /**
             *  Distance current + end of swing + distance direction + boolean improves
             */
            double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);
            double copCoMProjDistanceEndOFSwing = com2DtoProjectEndOfSwing.distance(desiredCMPtoProject);
            boolean distancePosAlignment = angleAndDistanceEvaluator.getDistancePosAlignment(com3D,desiredCMPtoProject,icpError);


            /**
             * Computation of time to position constraints
             */
            double tMinPosReachedPredicted = timeToConstraintsPredictor.getTMinPosReachedPredicted(z, dz, aMinCtrl, aMaxPredicted);
            yoTimeMinPosReached.set(tMinPosReachedPredicted);
            double tMaxPosReachedPredicted = timeToConstraintsPredictor.getTMaxPosReachedPredicted(z, dz, zMaxTouchDown, aMinPredicted, aMaxCtrl);
            yoTimeMaxPosReached.set(tMaxPosReachedPredicted);

            /**
             * Evaluate primary conditions
             */
            primaryCondition = primaryConditionEvaluator
                  .computeAndGetPrimaryConditionEnum(errorAngle,errorAngleEndOfSwing,negAlignTresh,posAlignTresh,useAngleForConditions,distancePosAlignment,
                                                     copCoMProjDistance,copCoMProjDistanceEndOFSwing,nonDynamicCase,copCoMProjMinDistance);
            primaryConditionYoEnum.set(primaryCondition);
               aMinCtrl = aMinCtrlAngle;
               aMaxCtrl = aMaxCtrlAngle;
               aMaxPredicted = fracOfAForPrediction * aMaxCtrl;
               aMinPredicted = fracOfAForPrediction * aMinCtrlAngle;

         }
         // set
         heightControlInThisWalkingState = true;
         calculateUseAngleForConditions = false;



         switch(primaryCondition)
         {
         case DEFAULT: desiredHeightAcceleration=linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
            break;
         case ALIGNED_POS: computePositiveAlignment();
            break;
         case PREPARE_NEG: computePrepare();
            break;
         }

         /**
          * Prevents from height variation when swing leg 'waits' for example
          */
         if(isInDoubleSupport&&(yoTimeInState.getDoubleValue()>walkingControllerParameters.getDefaultTransferTime())&&!nonDynamicCase || primaryCondition ==VaryingHeightPrimaryConditionEnum.DEFAULT)
         {
            primaryCondition = VaryingHeightPrimaryConditionEnum.DEFAULT;
            desiredHeightAcceleration=linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
         }

         /**
          * Acceleration and jerk checks, respectively
          */
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, aMinCtrl, aMaxCtrl);                                         // Acceleration
         desiredHeightAcceleration = MathTools
               .clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax * dt, desiredHeightAccelerationPreviousTick + jMax * dt); // Jerk

         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);

         /**
          * Added lin momentum rate based on CoP/ZMP
          */
         yoProjectedDesiredCMP.set(desiredCMPtoProject);
         addedHorizontalAcceleration.set(com3D);
         addedHorizontalAcceleration.sub(desiredCMPtoProject);
         addedHorizontalAcceleration.scale(yoDesiredHeightAcceleration.getDoubleValue() / com3D.getZ());
         modifiedLinearMomentumRateOfChange.set(linearMomentumRateOfChangeFromLIP.getX() + addedHorizontalAcceleration.getX() * totalMass,
                                                linearMomentumRateOfChangeFromLIP.getY() + addedHorizontalAcceleration.getY() * totalMass,
                                                yoDesiredHeightAcceleration.getDoubleValue() * totalMass);
      }
      else
      {
         primaryConditionYoEnum.set(VaryingHeightPrimaryConditionEnum.DEFAULT);
         desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ() / totalMass;

         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration,aMinCtrlAngle,aMaxCtrlAngle);
         desiredHeightAcceleration = MathTools
               .clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax * dt, desiredHeightAccelerationPreviousTick + jMax * dt); // Jerk
         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);
         modifiedLinearMomentumRateOfChange.setIncludingFrame(linearMomentumRateOfChangeFromLIP.getReferenceFrame(), linearMomentumRateOfChangeFromLIP.getX(),
                                                              linearMomentumRateOfChangeFromLIP.getY(), desiredHeightAcceleration * totalMass);
      }
   }

   private double getKneeAngle()
   {
      double angle;
      if (supportSide == null)
      {
         angle = 1.17;
         kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      }
      else
      {
         kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(supportSide, LegJointName.KNEE_PITCH);
         angle = kneeJoint.getQ();
      }
      return angle;
   }

   private double getzMax(boolean nonDynamicCase, double stateClock, double timeHalfWaySwing, boolean useAngleForConditions)
   {
      if(!useAngleForConditions && !nonDynamicCase)
      {
         zMax= zMaxStartSwing*1.01;
      }
      else if (nonDynamicCase)
      {
         zMax = zMaxStartSwing;
         yozMaxAlpha.set(0.0);
      }
      else if (stateClock > timeHalfWaySwing)
      {
         double alpha = (stateClock-tForHalfWaySwing)/(walkingControllerParameters.getDefaultSwingTime()-tForHalfWaySwing);
         alpha= Math.min(alpha,1);
         yozMaxAlpha.set(alpha);
         zMax = zMaxStartSwing - alpha*(zMaxStartSwing-zMaxTouchDown);
      }
      else
      {
         yozMaxAlpha.set(0.0);
         zMax = zMaxStartSwing;
      }
      return zMax;
   }

   private double getHeightAccelerationFromCircle()
   {
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      FramePoint3D hipPoint = new FramePoint3D(hipPitchFrame);
      FramePoint3D kneePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      kneePoint.changeFrame(hipPitchFrame);

      double thighLength = hipPoint.distance(kneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      kneePoint.setToZero(kneePitchFrame);
      FramePoint3D anklePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      anklePoint.changeFrame(kneePitchFrame);

      double shinLength = kneePoint.distance(anklePoint);
      anklePoint.changeFrame(ReferenceFrame.getWorldFrame());

      double r = shinLength + thighLength;
      double comXToAnkle = com3D.getX() - anklePoint.getX();
      double x = com3D.getX();
      double dx = comVelocity3D.getX();

      double rsqminxsq = r*r-comXToAnkle*comXToAnkle;
      double ddzCircle =  - 2*r*r*x*dx*dx*dx/(rsqminxsq*rsqminxsq);

      return  ddzCircle;
   }

   private void computePositiveAlignment()
   {
      if(z+0.5*Math.signum(dz)*dz*dz/-aMinPredicted<zMax && !firstDone)
      {
         desiredHeightAcceleration = aMaxCtrl;
      }
      else if(!secondDone && dz>0)
      {
         desiredHeightAcceleration = aMinCtrl;
         firstDone=true;
      }
      else
      {
         desiredHeightAcceleration = getHeightAccelerationFromCircle();
         if (nonDynamicCase)
         {
            double kp = walkingControllerParameters.getCoMHeightControlGains().getKp();
            double kd = walkingControllerParameters.getCoMHeightControlGains().getKd();
            desiredHeightAcceleration = kp * (zMax - z) - kd * dz;
         }
         secondDone=true;
      }
   }

   private void computePrepare()
   {
      double tFromMinToMax = timeToConstraintsPredictor.getTMaxPosReachedPredicted(zMin,0,zMaxTouchDown,aMinPredicted,aMaxCtrl);
      double tR = tRemainingEndOfWalkingState-tFromMinToMax;
      if(tR>0 && !firstDone)
      {
         double a = -0.5*tR*tR/-aMinPredicted;
         double b = 0.5*tR*tR-tR*dz/-aMinPredicted;
         double c = z-zMin + dz*tR - 0.5*dz*dz/-aMinPredicted;
         desiredHeightAcceleration= (-b+sqrt(b*b-4*a*c))/(2*a);
      }
      else if(tR<0 && !secondDone && z+0.5*Math.signum(dz)*dz*dz/-aMinPredicted<zMax)
      {
         desiredHeightAcceleration = aMaxCtrl;
         firstDone=true;
      }
      else
      {
         desiredHeightAcceleration = aMinCtrl;
         secondDone=true;
      }
   }

   public FrameVector3D getModifiedLinearMomentumRateOfChange()
   {
      return modifiedLinearMomentumRateOfChange;
   }

   public void setSupportPolygon(FrameConvexPolygon2D supportPolygon)
   {
      this.supportPolygon.setIncludingFrame(supportPolygon);
      this.supportPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
   }

   public void setDesiredCMP(FramePoint2D desiredCMP)
   {
      this.desiredCMPtoProject.setIncludingFrame(desiredCMP);
   }

   public void setICPError(FrameVector2D icpError)
   {
      this.icpError.setIncludingFrame(icpError);
   }

   public void setCoM(FramePoint3D CoM)
   {
      this.com2DtoProject.setIncludingFrame(CoM);
      this.com3D.setIncludingFrame(CoM);
   }

   public void setCoMEndOfSTep(FramePoint3D coMEndOfSTep)
   {
      yoCoMEndOfSwing2DNotHacky.set(coMEndOfSTep);
      com2DtoProjectEndOfSwing.setIncludingFrame(coMEndOfSTep);

   }

   public void setLinearMomentumRateOfChangeFromLIP(FrameVector3D linearMomentumRateOfChangeFromLIP)
   {
      this.linearMomentumRateOfChangeFromLIP.setIncludingFrame(linearMomentumRateOfChangeFromLIP);
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public void setIsInDoubleSupport(boolean isInDoubleSupport)
   {
      if (this.isInDoubleSupport == isInDoubleSupport)
      {
         walkingStateSwitch = false;
         this.isInDoubleSupport = isInDoubleSupport;
      }
      else
      {
         walkingStateSwitch = true;
         stateClock = 0;
         this.isInDoubleSupport = isInDoubleSupport;
      }
      yoWalkingStateSwitch.set(walkingStateSwitch);
   }

   public void setDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocity)
   {
      desiredICPVelocity.setIncludingFrame(desiredCapturePointVelocity);
      desiredICPVelocity.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public VaryingHeightPrimaryConditionEnum getPrimaryCondition()
   {
      return primaryConditionYoEnum.getEnumValue();
   }

   public double getTimeToSwitch()
   {
      return yoTimeToSwitch.getDoubleValue();
   }

   public double getTimeRemaining()
   {
      return yoTimeRemaining.getDoubleValue();
   }

   public double getErrorAngleEndOfSwing()
   {
      return copCoMICPeAngleFinal.getDoubleValue();
   }
}
