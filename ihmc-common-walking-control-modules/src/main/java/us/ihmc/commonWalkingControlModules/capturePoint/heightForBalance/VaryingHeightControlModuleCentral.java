package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.awt.*;

import static java.lang.Math.max;
import static java.lang.Math.sqrt;

public class VaryingHeightControlModuleCentral implements VaryingHeightControlModuleInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private WalkingControllerParameters walkingControllerParameters;

   private FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private FramePoint2D desiredCMPtoProject = new FramePoint2D();
   private FrameVector2D icpError = new FrameVector2D();
   private FramePoint2D com2DtoProject = new FramePoint2D();
   private FramePoint2D com2DtoProjectEndOfSwing = new FramePoint2D();
   private FramePoint3D com3D = new FramePoint3D();
   private FrameVector3D linearMomentumRateOfChangeFromLIP = new FrameVector3D();
   private FramePoint2D comEndOfStep2D = new FramePoint2D();
   private YoFramePoint2D yoCoMEndOfSTep2D;
   private YoFramePoint2D yoCoMEndOfSwing2DNotHacky;

   private FrameVector2D desiredICPVelocity = new FrameVector2D();

   private YoFramePoint2D yoProjectedDesiredCMP;
   private YoFramePoint2D yoProjectedCoM2D;
   private final YoEnum<VaryingHeightPrimaryConditionEnum> primaryConditionYoEnum;
   private final YoEnum<VaryingHeightSecondaryConditionEnum> secondaryConditionYoEnum;
   private boolean primaryConditionHasChanged;
   private boolean secondaryConditionHasChanged;
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

   private OneDoFJoint kneeJoint;
   private double kneeAngle;

   private boolean isInDoubleSupport;

   private YoDouble copCoMICPeAngle;
   private YoDouble copCoMICPeAngleFinal;
   private YoDouble copCoMDirectionAngle;

   private boolean useAngleForConditions;
   private YoBoolean yoUseAngleForConditions;

   private YoDouble yoTimeMinVelReached;
   private YoDouble yoTimeMaxVelReached;
   private YoDouble yoTimeMinPosReached;
   private YoDouble yoTimeMaxPosReached;
   private YoDouble yoTimeRemaining;
   private YoDouble yoTimeToSwitch;


   double posAlignTresh;
   double negAlignTresh;
   double vMax;
   double vMin;
   double aMaxCtrl;
   double aMinCtrl;
   double jMax;
   double jCtrl;
   double dt;
   double aMaxPredicted;
   double aMinPredicted;
   double zMax;
   double zMaxStartSwing;
   double zMaxTouchDown;
   double zMin;
   double minKneeAngle;
   double maxKneeAngle;
   double tf;
   double tForHalfWaySwing;
   double tRemainingEndOfWalkingState;
   double tRemainingConditionSwitch;
   double smoothEpsilon;

   boolean walkingStateSwitch;
   private YoBoolean yoWalkingStateSwitch;
   private YoBoolean yoAngleGrows;

   private YoBoolean yoAngleImproves;
   private YoBoolean yoDistanceImproves;

   private YoDouble yoPosAlignThresh;

   boolean heightControlInThisWalkingState = false;

   VaryingHeightAngleAndDistanceEvaluator angleAndDistanceEvaluator;
   VaryingHeightPrimaryConditionEvaluator primaryConditionEvaluator;
   VaryingHeightTimeToConstraintsPredictor timeToConstraintsPredictor;
   VaryingHeightSecondaryConditionEvaluator secondaryConditionEvaluator;


   public VaryingHeightControlModuleCentral(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry,
                                            YoGraphicsListRegistry yoGraphicsListRegistry, WalkingControllerParameters walkingControllerParameters)
   {
      this.controllerToolbox = controllerToolbox;
      this.totalMass = totalMass;
      this.walkingControllerParameters = walkingControllerParameters;
      parentRegistry.addChild(registry);
      dt = controllerToolbox.getControlDT();
      yoDesiredHeightAcceleration = new YoDouble("DesiredHeightAccelerationHeightControl", registry);

      // error angle, error angle end of swing and error angle direction
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      copCoMICPeAngleFinal = new YoDouble("CoPCoMICPeAngleFinal", registry);
      copCoMDirectionAngle = new YoDouble("CoPCoMDirectionAngle", registry);

      yoCoMEndOfSTep2D = new YoFramePoint2D("varyingHeightCoMEndOfSTep", ReferenceFrame.getWorldFrame(), registry);
      yoCoMEndOfSwing2DNotHacky = new YoFramePoint2D("varyingHeightCoMEndOfSwingFromPlanner", ReferenceFrame.getWorldFrame(), registry);

      // angle and distance booleans
      yoAngleGrows = new YoBoolean("angleGrows", registry);
      yoAngleImproves = new YoBoolean("angleImproves", registry);
      yoDistanceImproves = new YoBoolean("distanceImproves", registry);
      yoUseAngleForConditions = new YoBoolean("useAngleForConditions", registry);

      yoWalkingStateSwitch = new YoBoolean("stateSwitch",registry);

      // estimated times
      yoTimeMinVelReached = new YoDouble("estTimeMinVel", registry);
      yoTimeMaxVelReached = new YoDouble("estTimeMaxVel", registry);
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);
      yoTimeToSwitch = new YoDouble("estTimeToSwitch",registry);

      // threshold for positive alignment, can be modifed
      yoPosAlignThresh = new YoDouble("posAlignmentThreshold",registry);

      primaryConditionYoEnum = new YoEnum<>("varyingHeightCondition", registry, VaryingHeightPrimaryConditionEnum.class);
      secondaryConditionYoEnum = new YoEnum<>("varyingHeightSecondaryCondition", registry, VaryingHeightSecondaryConditionEnum.class);

      yoTimeInState = new YoDouble("varyingHeightTimeInState", registry);

      String label = getClass().getSimpleName();
      ArtifactList artifacts = new ArtifactList(label);

      yoProjectedDesiredCMP = new YoFramePoint2D(label + "projCMPd", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CMPd", yoProjectedDesiredCMP.getYoX(), yoProjectedDesiredCMP.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED,
                                           0.05));

      yoProjectedCoM2D = new YoFramePoint2D(label + "projCOM2D", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CoM2D", yoProjectedCoM2D.getYoX(), yoProjectedCoM2D.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.02));


      artifacts.setVisible(true);
      yoGraphicsListRegistry.registerArtifactList(artifacts);
      // Parameters:
      vMax = 0.7;
      vMin = -0.6;
      zMaxStartSwing = 1.17;
      zMaxTouchDown = 1.11;
      zMin = 1.00;
      smoothEpsilon = 0.05;
      minKneeAngle = 0.15;
      maxKneeAngle = 2.1;

      // Parameters that can be modified:
      aMaxCtrl = 5;
      aMinCtrl = -5;
      jMax = 200;
      zMax = 1.17;
      aMaxPredicted = 0.6 * aMaxCtrl;
      aMinPredicted = 0.6 * aMinCtrl;
      posAlignTresh = 0.7;
      negAlignTresh = Math.PI - 0.9;
      tForHalfWaySwing = 0.416*walkingControllerParameters.getDefaultSwingTime();

      angleAndDistanceEvaluator = new VaryingHeightAngleAndDistanceEvaluator();
      primaryConditionEvaluator = new VaryingHeightPrimaryConditionEvaluator(zMin, minKneeAngle, maxKneeAngle);
      timeToConstraintsPredictor = new VaryingHeightTimeToConstraintsPredictor(zMin,vMin,vMax);
      secondaryConditionEvaluator = new VaryingHeightSecondaryConditionEvaluator(zMin,tForHalfWaySwing,smoothEpsilon,
                                                                                 timeToConstraintsPredictor);
   }

   public void compute()
   {
      stateClock = stateClock + dt;
      tf = walkingControllerParameters.getDefaultSwingTime();
      tRemainingEndOfWalkingState = tf - stateClock;
      yoTimeRemaining.set(tRemainingEndOfWalkingState);
      yoTimeInState.set(stateClock);

      VaryingHeightPrimaryConditionEnum primaryConditionPreviousTick = primaryConditionYoEnum.getEnumValue();
      VaryingHeightSecondaryConditionEnum secondaryConditionPreviousTick = secondaryConditionYoEnum.getEnumValue();
      desiredHeightAccelerationPreviousTick = desiredHeightAcceleration;

      kneeAngle = getKneeAngle();

      com2DtoProject.changeFrame(ReferenceFrame.getWorldFrame());
      com2DtoProjectEndOfSwing.changeFrame(ReferenceFrame.getWorldFrame());
      com3D.changeFrame(ReferenceFrame.getWorldFrame());
      desiredCMPtoProject.changeFrame(ReferenceFrame.getWorldFrame());
      icpError.changeFrame(ReferenceFrame.getWorldFrame());

      yoCoMEndOfSTep2D.set(comEndOfStep2D);
      double distance = supportPolygon.signedDistance(desiredCMPtoProject);
      boolean cmpOutsidePolygon = (!supportPolygon.isPointInside(desiredCMPtoProject));
      boolean isProjected = supportPolygon.orthogonalProjection(desiredCMPtoProject);
      yoProjectedDesiredCMP.set(desiredCMPtoProject);

      // For standing push
      boolean nonDynamicCase = desiredICPVelocity.length()<0.02;

      // Reset values if state switch occurs, or if standing
      if (walkingStateSwitch || nonDynamicCase)
      {
         heightControlInThisWalkingState = false;
         posAlignTresh = 0.7;
         negAlignTresh = Math.PI - 1.0;
      }

      // Determines if height control or not
      boolean heightControlCondition = (
            isProjected  &&cmpOutsidePolygon&& (distance > 0.7 * walkingControllerParameters.getMaxAllowedDistanceCMPSupport()) && (isInDoubleSupport == false || nonDynamicCase==true)
                  || heightControlInThisWalkingState == true);

      // Some variables
      FrameVector3D centerOfMassVelocity = new FrameVector3D();
      controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);
      double z = com3D.getZ();
      double dz = centerOfMassVelocity.getZ();
      FrameVector2D centerOfMassVelocity2D = new FrameVector2D();
      centerOfMassVelocity2D.setIncludingFrame(centerOfMassVelocity);
      centerOfMassVelocity2D.changeFrame(ReferenceFrame.getWorldFrame());

      // Determines if the decision has to be made which control law has to be used.
      calculateUseAngleForConditions = (isProjected  && cmpOutsidePolygon && heightControlInThisWalkingState == false);


      // HEIGHT CONTROL
      if (heightControlCondition)
      {
         // different max heights
         if(nonDynamicCase)
         {
            zMax = zMaxStartSwing;
         }
         else if (stateClock > tForHalfWaySwing)
         {
            zMax = zMaxTouchDown;
         }
         else
         {
            zMax = zMaxStartSwing;
         }

         // Min/max - control law set based on using angle or distance
         if(calculateUseAngleForConditions || walkingStateSwitch)
         {
            useAngleForConditions = angleAndDistanceEvaluator.getUseAngleForConditions(supportPolygon, desiredCMPtoProject, yoCoMEndOfSwing2DNotHacky);
            yoUseAngleForConditions.set(useAngleForConditions);
            if(!useAngleForConditions )
            {
            /*
            aMinCtrl=2*(zMin-z)/(tRemainingEndOfWalkingState*tRemainingEndOfWalkingState);
            aMinCtrl=MathTools.clamp(aMinCtrl,-1.5,0);
            aMaxCtrl=2*(zMin-z)/(tRemainingEndOfWalkingState*tRemainingEndOfWalkingState);
            aMaxPredicted = 0.8 * aMaxCtrl;
            aMinPredicted = 0.8 * aMinCtrl;
            */
               aMinCtrl=-1.0;
               aMaxCtrl=3.0;
               aMaxPredicted = 0.6 * aMaxCtrl;
               aMinPredicted = 0.6 * -3;
            }
            else if(useAngleForConditions)
            {
               aMinCtrl=-5;
               aMaxCtrl=5;
               aMaxPredicted = 0.6 * aMaxCtrl;
               aMinPredicted = 0.6 * aMinCtrl;
            }

         }
         // set
         heightControlInThisWalkingState = true;
         calculateUseAngleForConditions=false;

         // projected com positions, NOT USED
         FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProjectEndOfSwing);
         yoProjectedCoM2D.set(com2DtoProject);



         /**
          * Error angle current
          */
         double errorAngle = angleAndDistanceEvaluator.getErrorAngle(icpError,desiredCMPtoProject,com3D);
         copCoMICPeAngle.set(errorAngle);
         /**
          * Angle direction
          */
         boolean angleGrows = angleAndDistanceEvaluator.getAngleGrows(errorAngle,centerOfMassVelocity2D,desiredCMPtoProject,com3D);
         yoAngleGrows.set(angleGrows);
         /**
          * If angle improves or not
          */
         boolean angleImproves = angleAndDistanceEvaluator.getAngleImproves(angleGrows,errorAngle,posAlignTresh,negAlignTresh);
         yoAngleImproves.set(angleImproves);

         /**
          * Error angle end of swing
          */
         double errorAngleEndOfSwing = angleAndDistanceEvaluator.getErrorAngleEndOfSwing(yoCoMEndOfSwing2DNotHacky,desiredCMPtoProject,icpError);
         copCoMICPeAngleFinal.set(errorAngleEndOfSwing);

         /**
          *  Distance current + end of swing + distance direction + boolean improves
          */
         double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);
         double copCoMProjDistanceEndOFSwing = com2DtoProjectEndOfSwing.distance(desiredCMPtoProject);
         boolean distanceImproves = angleAndDistanceEvaluator.getDistanceImproves(centerOfMassVelocity2D,icpError);
         //boolean distancePosAlignment = angleAndDistanceEvaluator.getDistancePosAlignment(errorAngle);
         FrameVector2D copCoMVec = new FrameVector2D();
         copCoMVec.setIncludingFrame(com3D);
         copCoMVec.sub(desiredCMPtoProject);
         boolean distancePosAlignment = (MathTools.sign(copCoMVec.getY())==MathTools.sign(icpError.getY()));
         yoDistanceImproves.set(distanceImproves);






         /**
          * Computation of time to velocity constraints
          */
         double tMinVelReachedPredicted = timeToConstraintsPredictor.getTMinVelReachedPredicted(dz, aMinPredicted);
         yoTimeMinVelReached.set(tMinVelReachedPredicted);
         double tMaxVelReachedPredicted = timeToConstraintsPredictor.getTMaxVelReachedPredicted(dz, aMaxPredicted);
         yoTimeMaxVelReached.set(tMaxVelReachedPredicted);

         /**
          * Computation of time to position constraints
          */
         double tMinPosReachedPredicted = timeToConstraintsPredictor.getTMinPosReachedPredicted(z,dz,aMinPredicted,aMaxPredicted);
         yoTimeMinPosReached.set(tMinPosReachedPredicted);
         double tMaxPosReachedPredicted = timeToConstraintsPredictor.getTMaxPosReachedPredicted(z,dz,zMaxTouchDown,aMinPredicted,aMaxPredicted);
         yoTimeMaxPosReached.set(tMaxPosReachedPredicted);



         /**
          * Evaluate primary conditions
          */
         VaryingHeightPrimaryConditionEnum primaryCondition = primaryConditionEvaluator.computeAndGetPrimaryConditionEnum(aMinPredicted,aMaxPredicted,z,dz,zMax,kneeAngle,errorAngle,
                                                                                                                          errorAngleEndOfSwing,angleGrows,negAlignTresh,
                                                                                                                          posAlignTresh,primaryConditionPreviousTick,useAngleForConditions, distancePosAlignment,
                                                                                                                          copCoMProjDistance, nonDynamicCase);
         primaryConditionYoEnum.set(primaryCondition);
         primaryConditionHasChanged = primaryConditionEvaluator.getPrimaryConditionHasChanged();

         // In second phase of swing, always the most extreme minimal control (needs to 'fall' with the circle the leg describes)
         if(primaryCondition==VaryingHeightPrimaryConditionEnum.MAXZ)
         {
            aMinCtrl = -5;
         }

         /**
          * Parameters for control and secondary condition
          */
         VaryingHeightSecondaryConditionEnum secondaryCondition = secondaryConditionEvaluator.computeAndGetSecondaryConditionEnum(aMinCtrl,aMaxCtrl,z, dz,primaryCondition,primaryConditionHasChanged,
                                                                                                                                  secondaryConditionPreviousTick, stateClock,tMinVelReachedPredicted,tMinPosReachedPredicted,
                                                                                                                                  tMaxVelReachedPredicted,tMaxPosReachedPredicted,tRemainingEndOfWalkingState,errorAngle,
                                                                                                                                  errorAngleEndOfSwing,angleGrows,posAlignTresh, negAlignTresh,
                                                                                                                                  zMaxTouchDown, nonDynamicCase);
         secondaryConditionYoEnum.set(secondaryCondition);
         posAlignTresh = secondaryConditionEvaluator.getModifiedPosAlignTresh();
         yoPosAlignThresh.set(posAlignTresh);
         tRemainingConditionSwitch =secondaryConditionEvaluator.getTimeToConditionSwitch();
         yoTimeToSwitch.set(tRemainingConditionSwitch);
         double tConst = secondaryConditionEvaluator.getTimeToClosestConstraint();
         double aCtrl = secondaryConditionEvaluator.getControlBound();

         double aSmooth = secondaryConditionEvaluator.getControlSmooth();

         /**
          * Control
          */
         if (secondaryCondition == VaryingHeightSecondaryConditionEnum.SMOOTH)
         {
            desiredHeightAcceleration = aSmooth;//tConst*aCtrl/tRemainingConditionSwitch;
         }
         else if (secondaryCondition == VaryingHeightSecondaryConditionEnum.HOLD)
         {
            desiredHeightAcceleration = 0;
         }
         else if (secondaryCondition == VaryingHeightSecondaryConditionEnum.DEFAULT)
         {
            desiredHeightAcceleration = aCtrl;
         }



         /**
          * Velocity, acceleration and jerk checks, respectively
          */
         //desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, (vMin -dz)/dt, (vMax -dz)/dt);                      // Velocity
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, aMinCtrl, aMaxCtrl);                                         // Acceleration
         desiredHeightAcceleration = MathTools
               .clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax * dt, desiredHeightAccelerationPreviousTick + jMax * dt); // Jerk

         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);

         /**
          * Added lin momentum rate based on CoP/ZMP
          */
         addedHorizontalAcceleration.set(com3D);
         addedHorizontalAcceleration.sub(desiredCMPtoProject);
         addedHorizontalAcceleration.scale(yoDesiredHeightAcceleration.getDoubleValue() / com3D.getZ());
         modifiedLinearMomentumRateOfChange.set(linearMomentumRateOfChangeFromLIP.getX() + addedHorizontalAcceleration.getX() * totalMass,
                                                linearMomentumRateOfChangeFromLIP.getY() + addedHorizontalAcceleration.getY() * totalMass,
                                                yoDesiredHeightAcceleration.getDoubleValue() * totalMass);
      }
      else
      {
         /**
          * Smoothing out differences if varying height controller kicked in, horizontal linear momentum rate is here not affected by height acceleration
          * (as in normal control setting)
          */
         primaryConditionYoEnum.set(VaryingHeightPrimaryConditionEnum.DEFAULT);
         secondaryConditionYoEnum.set(VaryingHeightSecondaryConditionEnum.DEFAULT);
         desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ() / totalMass;
         desiredHeightAcceleration = MathTools
               .clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax * dt, desiredHeightAccelerationPreviousTick + jMax * dt); // Jerk
         yoDesiredHeightAcceleration.set(desiredHeightAcceleration); // HERE DOESNT AFFECT Ld!!
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
      }
      else
      {
         kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(supportSide, LegJointName.KNEE_PITCH);
         angle = kneeJoint.getQ();
      }
      return angle;
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
}
