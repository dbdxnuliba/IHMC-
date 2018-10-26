package us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.*;
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

   private YoFramePoint2D yoProjectedDesiredCMP;
   private YoFramePoint2D yoProjectedCoM2D;
   private final YoEnum<VaryingHeightPrimaryConditionEnum> primaryConditionYoEnum;
   private final YoEnum<VaryingHeightSecondaryConditionEnum> secondaryConditionYoEnum;
   private boolean primaryConditionHasChanged;
   private boolean secondaryConditionHasChanged;

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

   private YoDouble yoCoMVelocity;

   double posAlignTresh;
   double negAlignTresh;
   double vMax;
   double vMin;
   double aMaxCtrl;
   double aMinCtrl;
   double jMax;
   double dt;
   double aMaxPredicted;
   double aMinPredicted;
   double zMax;
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
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      copCoMICPeAngleFinal = new YoDouble("CoPCoMICPeAngleFinal", registry);
      copCoMDirectionAngle = new YoDouble("CoPCoMDirectionAngle", registry);

      yoCoMEndOfSTep2D = new YoFramePoint2D("varyingHeightCoMEndOfSTep", ReferenceFrame.getWorldFrame(), registry);
      yoCoMEndOfSwing2DNotHacky = new YoFramePoint2D("varyingHeightCoMEndOfSwingFromPlanner", ReferenceFrame.getWorldFrame(), registry);

      yoAngleGrows = new YoBoolean("angleGrows", registry);
      yoAngleImproves = new YoBoolean("angleImproves", registry);
      yoDistanceImproves = new YoBoolean("distanceImproves", registry);
      yoUseAngleForConditions = new YoBoolean("useAngleForConditions", registry);

      yoTimeMinVelReached = new YoDouble("estTimeMinVel", registry);
      yoTimeMaxVelReached = new YoDouble("estTimeMaxVel", registry);
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);

      yoPosAlignThresh = new YoDouble("posAlignmentThreshold",registry);

      yoCoMVelocity = new YoDouble("comVelocityHeightControl", registry);

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
      vMax = 0.7;
      vMin = -0.6;
      aMaxCtrl = 5;
      aMinCtrl = -5;
      jMax = 200;
      aMaxPredicted = 0.6 * aMaxCtrl;
      aMinPredicted = 0.6 * aMinCtrl;
      zMax = 1.17;
      zMaxTouchDown = 1.12;
      zMin = 1.00;
      smoothEpsilon = 0.03;
      minKneeAngle = 0.25;
      maxKneeAngle = 2.1;
      posAlignTresh = 0.7;
      negAlignTresh = Math.PI - 1.0;
      tForHalfWaySwing = 0.416*walkingControllerParameters.getDefaultSwingTime();

      angleAndDistanceEvaluator = new VaryingHeightAngleAndDistanceEvaluator();
      primaryConditionEvaluator = new VaryingHeightPrimaryConditionEvaluator(zMin, minKneeAngle, maxKneeAngle, aMinPredicted, aMaxPredicted);
      timeToConstraintsPredictor = new VaryingHeightTimeToConstraintsPredictor(zMin,vMin,vMax,aMinPredicted,aMaxPredicted);
      secondaryConditionEvaluator = new VaryingHeightSecondaryConditionEvaluator(zMin,aMinCtrl,aMaxCtrl,tForHalfWaySwing,smoothEpsilon,
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
      double distance = supportPolygon.distance(desiredCMPtoProject);
      boolean cmpOutsidePolygon = !supportPolygon.isPointInside(desiredCMPtoProject);
      LineSegment2D projectedCMPPolygonEdge = new LineSegment2D();
      boolean closestEdgeFound = supportPolygon.getClosestEdge(desiredCMPtoProject, projectedCMPPolygonEdge);
      int projectedCMPPolygonEdgeIndex = supportPolygon.getClosestEdgeIndex(desiredCMPtoProject);
      boolean isProjected = supportPolygon.orthogonalProjection(desiredCMPtoProject);

      yoProjectedDesiredCMP.set(desiredCMPtoProject);
      yoDesiredHeightAcceleration.set(0.0);

      boolean heightControlCondition = (
            isProjected && cmpOutsidePolygon && distance > 0.7 * walkingControllerParameters.getMaxAllowedDistanceCMPSupport() && isInDoubleSupport == false
                  || heightControlInThisWalkingState == true);

      if (heightControlCondition)
      {
         heightControlInThisWalkingState = true;
         if (walkingStateSwitch)
         {
            heightControlInThisWalkingState = false;
            posAlignTresh = 0.7;
            negAlignTresh = Math.PI - 1.0;
         }
         FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProjectEndOfSwing);
         yoProjectedCoM2D.set(com2DtoProject);

         FrameVector3D centerOfMassVelocity = new FrameVector3D();
         controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);

         double z = com3D.getZ();
         double dz = centerOfMassVelocity.getZ();
         yoCoMVelocity.set(dz);

         FrameVector2D centerOfMassVelocity2D = new FrameVector2D();
         centerOfMassVelocity2D.setIncludingFrame(centerOfMassVelocity);
         centerOfMassVelocity2D.changeFrame(ReferenceFrame.getWorldFrame());

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
          *  Distance current + end ofswing + distance direction + boolean improves
          */
         double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);
         double copCoMProjDistanceEndOFSwing = com2DtoProjectEndOfSwing.distance(desiredCMPtoProject);
         boolean distanceImproves = angleAndDistanceEvaluator.getDistanceImproves(centerOfMassVelocity2D,icpError);
         boolean distancePosAlignment = angleAndDistanceEvaluator.getDistancePosAlignment(errorAngle);
         yoDistanceImproves.set(distanceImproves);
         useAngleForConditions = angleAndDistanceEvaluator.getUseAngleForConditions(supportPolygon, desiredCMPtoProject);
         yoUseAngleForConditions.set(useAngleForConditions);

         /**
          * Different max heights
          */
         if (stateClock > tForHalfWaySwing)
         {
            zMax = zMaxTouchDown;
         }
         else
         {
            zMax = 1.17;
         }

         /**
          * Computation of time to velocity constraints
          */
         double tMinVelReachedPredicted = timeToConstraintsPredictor.getTMinVelReachedPredicted(dz);
         yoTimeMinVelReached.set(tMinVelReachedPredicted);
         double tMaxVelReachedPredicted = timeToConstraintsPredictor.getTMaxVelReachedPredicted(dz);
         yoTimeMaxVelReached.set(tMaxVelReachedPredicted);

         /**
          * Computation of time to position constraints
          */
         double tMinPosReachedPredicted = timeToConstraintsPredictor.getTMinPosReachedPredicted(z,dz);
         yoTimeMinPosReached.set(tMinPosReachedPredicted);
         double tMaxPosReachedPredicted = timeToConstraintsPredictor.getTMaxPosReachedPredicted(z,dz,zMaxTouchDown);
         yoTimeMaxPosReached.set(tMaxPosReachedPredicted);


         /**
          * Evaluate primary conditions
          */
         VaryingHeightPrimaryConditionEnum primaryCondition = primaryConditionEvaluator.computeAndGetPrimaryConditionEnum(z,dz,zMax,kneeAngle,errorAngle,
                                                                                                                          errorAngleEndOfSwing,angleGrows,negAlignTresh,
                                                                                                                          posAlignTresh,primaryConditionPreviousTick,useAngleForConditions, copCoMProjDistance);
         primaryConditionYoEnum.set(primaryCondition);
         primaryConditionHasChanged = primaryConditionEvaluator.getPrimaryConditionHasChanged();

         /**
          * Parameters for control and secondary condition
          */
         VaryingHeightSecondaryConditionEnum secondaryCondition = secondaryConditionEvaluator.computeAndGetSecondaryConditionEnum(z, dz,primaryCondition,primaryConditionHasChanged,
                                                                                                                                  secondaryConditionPreviousTick, stateClock,tMinVelReachedPredicted,tMinPosReachedPredicted,
                                                                                                                                  tMaxVelReachedPredicted,tMaxPosReachedPredicted,tRemainingEndOfWalkingState,errorAngle,
                                                                                                                                  errorAngleEndOfSwing,angleGrows,posAlignTresh, negAlignTresh,
                                                                                                                                  zMaxTouchDown);
         secondaryConditionYoEnum.set(secondaryCondition);
         posAlignTresh = secondaryConditionEvaluator.getModifiedPosAlignTresh();
         yoPosAlignThresh.set(posAlignTresh);
         tRemainingConditionSwitch =secondaryConditionEvaluator.getTimeToConditionSwitch();
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

         addedHorizontalAcceleration.set(com3D);
         addedHorizontalAcceleration.sub(desiredCMPtoProject);
         addedHorizontalAcceleration.scale(yoDesiredHeightAcceleration.getDoubleValue() / com3D.getZ());
         modifiedLinearMomentumRateOfChange.set(linearMomentumRateOfChangeFromLIP.getX() + addedHorizontalAcceleration.getX() * totalMass,
                                                linearMomentumRateOfChangeFromLIP.getY() + addedHorizontalAcceleration.getY() * totalMass,
                                                yoDesiredHeightAcceleration.getDoubleValue() * totalMass);
      }
      else
      {
         FrameVector3D centerOfMassVelocity = new FrameVector3D();
         controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);
         /**
          * Smoothing out differences if varying height controller kicked in
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
   }
}