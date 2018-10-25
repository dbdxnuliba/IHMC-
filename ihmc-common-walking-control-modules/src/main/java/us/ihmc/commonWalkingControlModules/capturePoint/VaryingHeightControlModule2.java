package us.ihmc.commonWalkingControlModules.capturePoint;

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

public class VaryingHeightControlModule2 implements VaryingHeightControlModuleInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private WalkingControllerParameters walkingControllerParameters;
   private enum VaryingHeightCondition {ALIGNED_NEG, ALIGNED_POS, MAXZ, MINZ, GO_BACK, DEFAULT, PREPARE_NEG, PREPARE_POS};
   private enum VaryingHeightSecondaryCondition {HOLD, SMOOTH, DEFAULT }
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
   private final YoEnum<VaryingHeightCondition> varyingHeightConditionYoEnum;
   private final YoEnum<VaryingHeightSecondaryCondition> varyingHeightSecondaryConditionYoEnum;
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

   private FrameVector3D  modifiedLinearMomentumRateOfChange = new FrameVector3D();

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
   double tr;
   double smoothEpsilon;

   boolean walkingStateSwitch;
   private YoBoolean yoAngleGrows;

   private YoBoolean yoAngleImproves;
   private YoBoolean yoDistanceImproves;

   boolean heightControlInThisWalkingState = false;


   public  VaryingHeightControlModule2(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, WalkingControllerParameters walkingControllerParameters)
   {
      this.controllerToolbox=controllerToolbox;
      this.totalMass=totalMass;
      this.walkingControllerParameters=walkingControllerParameters;
      parentRegistry.addChild(registry);
      dt = controllerToolbox.getControlDT();
      yoDesiredHeightAcceleration = new YoDouble("DesiredHeightAccelerationHeightControl", registry);
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      copCoMICPeAngleFinal = new YoDouble("CoPCoMICPeAngleFinal",registry);
      copCoMDirectionAngle = new YoDouble("CoPCoMDirectionAngle",registry);

      yoCoMEndOfSTep2D = new YoFramePoint2D("varyingHeightCoMEndOfSTep",ReferenceFrame.getWorldFrame(), registry);
      yoCoMEndOfSwing2DNotHacky = new YoFramePoint2D("varyingHeightCoMEndOfSwingFromPlanner", ReferenceFrame.getWorldFrame(),registry);

      yoAngleGrows = new YoBoolean("angleGrows", registry);
      yoAngleImproves = new YoBoolean("angleImproves", registry);
      yoDistanceImproves = new YoBoolean("distanceImproves",registry);
      yoUseAngleForConditions = new YoBoolean("useAngleForConditions",registry);

      yoTimeMinVelReached = new YoDouble("estTimeMinVel", registry);
      yoTimeMaxVelReached = new YoDouble("estTimeMaxVel", registry);
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);

      yoCoMVelocity = new YoDouble("comVelocityHeightControl", registry);

      varyingHeightConditionYoEnum = new YoEnum<>("varyingHeightCondition", registry, VaryingHeightCondition.class);
      varyingHeightSecondaryConditionYoEnum = new YoEnum<>("varyingHeightSecondaryCondition", registry, VaryingHeightSecondaryCondition.class);

      yoTimeInState = new YoDouble("varyingHeightTimeInState", registry);

      String label = getClass().getSimpleName();
      ArtifactList artifacts = new ArtifactList(label);

      yoProjectedDesiredCMP = new YoFramePoint2D(label + "projCMPd", ReferenceFrame.getWorldFrame(),registry);
      artifacts.add(new YoArtifactPosition("proj CMPd", yoProjectedDesiredCMP.getYoX(), yoProjectedDesiredCMP.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.05));

      yoProjectedCoM2D = new YoFramePoint2D(label + "projCOM2D", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CoM2D", yoProjectedCoM2D.getYoX(), yoProjectedCoM2D.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.02));

      artifacts.setVisible(true);
      yoGraphicsListRegistry.registerArtifactList(artifacts);
      vMax = 0.7;
      vMin = -0.6;
      aMaxCtrl = 5;
      aMinCtrl = -5;
      jMax = 200;
      aMaxPredicted = 0.6* aMaxCtrl;
      aMinPredicted = 0.6* aMinCtrl;
      zMax = 1.17;
      zMaxTouchDown = 1.14;
      zMin = 1.00;
      minKneeAngle = 0.25;
      maxKneeAngle = 2.1;
      posAlignTresh = 0.7;
      negAlignTresh = Math.PI-1.0;
      smoothEpsilon = 0.03;
   }

   public void compute()
   {
      stateClock=stateClock+dt;
      yoTimeInState.set(stateClock);

      VaryingHeightCondition varyingHeightConditionPreviousTick = varyingHeightConditionYoEnum.getEnumValue();
      VaryingHeightSecondaryCondition varyingHeightSecondaryConditionPreviousTick = varyingHeightSecondaryConditionYoEnum.getEnumValue();
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

      boolean heightControlCondition = (isProjected && cmpOutsidePolygon  && distance>0.7*walkingControllerParameters.getMaxAllowedDistanceCMPSupport() && isInDoubleSupport==false || heightControlInThisWalkingState==true);

      if(heightControlCondition)
      {
         heightControlInThisWalkingState= true;
         if(walkingStateSwitch)
         {
            heightControlInThisWalkingState=false;
            posAlignTresh = 0.7;
            negAlignTresh = Math.PI-1.0;
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
         double errorAngle = getErrorAngle();
         copCoMICPeAngle.set(errorAngle);
         /**
          * Angle direction
          */
         boolean angleGrows = getAngleGrows(errorAngle, centerOfMassVelocity2D);
         yoAngleGrows.set(angleGrows);
         /**
          * If angle improves or not
          */
         boolean angleImproves = getAngleImproves(angleGrows,errorAngle);
         yoAngleImproves.set(angleImproves);

         /**
          * Error angle end of swing
          */
         double errorAngleEndOfSwing = getErrorAngleEndOfSwing();

         /**
          *  Distance current + end ofswing + distance direction + boolean improves
          */
         double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);
         double copCoMProjDistanceEndOFSwing = com2DtoProjectEndOfSwing.distance(desiredCMPtoProject);
         boolean distanceImproves = getDistanceImproves(centerOfMassVelocity2D);
         boolean distancePosAlignment = getDistancePosAlignment(errorAngle);
         yoDistanceImproves.set(distanceImproves);


         tf = walkingControllerParameters.getDefaultSwingTime();
         tr = tf - stateClock ;
         FramePoint2D closestVertex = new FramePoint2D();
         supportPolygon.getClosestVertex(desiredCMPtoProject, closestVertex);
            if( MathTools.epsilonEquals(projectedCMPPolygonEdge.getFirstEndpointX(), projectedCMPPolygonEdge.getSecondEndpointX(), 0.05) )
            {
               useAngleForConditions = true;
            }
            else if(closestVertex.distance(desiredCMPtoProject)<0.02)
            {
               useAngleForConditions = true;
            }
            else
            {
               useAngleForConditions = false;
            }

         yoUseAngleForConditions.set(useAngleForConditions);
         yoTimeRemaining.set(tr);

         /**
          * Different max heights
          */
         if(isInDoubleSupport==false && stateClock>0.25)
         {
            zMax=zMaxTouchDown;
            if(dz>0)
            {
               zMax=1.1;
            }
         }
         else if (isInDoubleSupport==true)
         {
            zMax = 1.16;
         }
         else
         {
            zMax=1.18;
         }



         /**
          * Computation of time to velocity constraints
          */
         double tMinVelReachedPredicted = (vMin-dz)/aMinPredicted;
         tMinVelReachedPredicted = Math.max(0,tMinVelReachedPredicted);
         yoTimeMinVelReached.set(tMinVelReachedPredicted);
         double tMaxVelReachedPredicted = (vMax-dz)/aMaxPredicted;
         tMaxVelReachedPredicted = Math.max(0,tMaxVelReachedPredicted);
         yoTimeMaxVelReached.set(tMaxVelReachedPredicted);

         /**
          * Computation of time to position constraints
          */
         double tMaxPosReachedPredicted = getTMaxPosReachedPredicted(z,dz);
         yoTimeMaxPosReached.set(tMaxPosReachedPredicted);
         double tMinPosReachedPredicted = getTMinPosReachedPredicted(z,dz);
         yoTimeMinPosReached.set(tMinPosReachedPredicted);

         double tMinPred;
         if(tMinVelReachedPredicted<tMinPosReachedPredicted)
         {
            tMinPred = tMinVelReachedPredicted;
         }
         else
         {
            tMinPred = tMinPosReachedPredicted;
         }

         double tMaxPred;
         if(tMaxVelReachedPredicted<tMaxPosReachedPredicted)
         {
            tMaxPred = tMaxVelReachedPredicted;
         }
         else
         {
            tMaxPred = tMaxPosReachedPredicted;
         }

         /**
          * Evaluate primary conditions
          */
         evaluatePrimaryCondition(z,dz,errorAngle,errorAngleEndOfSwing,angleGrows,varyingHeightConditionPreviousTick);

         /**
          * Gather results from primary conditions
          */
         if(varyingHeightConditionYoEnum.getEnumValue()!=varyingHeightConditionPreviousTick)
         {
            primaryConditionHasChanged=true;
         }
         else
         {
            primaryConditionHasChanged=false;
         }
         double aCtrl = 0;
         double tConst =0;
         if(varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.ALIGNED_NEG||varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_NEG||varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.MAXZ)
         {
            aCtrl = aMinCtrl;
            tConst = tMinPred;
         }
         if(varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.ALIGNED_POS || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.MINZ || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_POS)
         {
            aCtrl = aMaxCtrl;
            tConst = tMaxPred;
         }

         if(varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_NEG)
         {
            double trStep= tr;
            if (angleGrows)
            {
               tr = (-posAlignTresh - errorAngle) / (errorAngleEndOfSwing - errorAngle) * tr;
            }
            else
            {
               tr = (posAlignTresh - errorAngle) / (errorAngleEndOfSwing - errorAngle) * tr;
            }

            if(trStep-tr>getTMaxPosReachedPredicted(zMin, 0))
            {
               posAlignTresh = getTMaxPosReachedPredicted(zMin,0)/(trStep-tr)*posAlignTresh;
            }
         }
         else if(varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_POS)
         {
            if (angleGrows)
            {
               tr = (-negAlignTresh-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
            }
            else
            {
               tr = (negAlignTresh-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
            }
         }

         /**
          * Evaluate secondary conditions
          */
         if(((stateClock>0.25 && MathTools.epsilonEquals(tConst,tr,smoothEpsilon)) || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_POS || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_NEG) && tConst<tr)
         {
            varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
         }
         else if(stateClock>0.25 && tConst<tr)
         {
            varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.HOLD);
            if(varyingHeightSecondaryConditionPreviousTick==VaryingHeightSecondaryCondition.SMOOTH)
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
            }
         }
         else
         {
            varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.DEFAULT);
            if(varyingHeightSecondaryConditionPreviousTick==VaryingHeightSecondaryCondition.HOLD)
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.HOLD);
            }
            else if(!primaryConditionHasChanged && varyingHeightSecondaryConditionPreviousTick==VaryingHeightSecondaryCondition.SMOOTH)
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
            }
         }



         /**
          * Control
          */
         if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.SMOOTH) {desiredHeightAcceleration = (tConst/tr)*aCtrl;}
         else if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.HOLD) {desiredHeightAcceleration = 0;}
         else if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.DEFAULT) {desiredHeightAcceleration = aCtrl;}




         /**
          * Velocity, acceleration and jerk checks, respectively
          */
         //desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, (vMin -dz)/dt, (vMax -dz)/dt);                      // Velocity
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, aMinCtrl, aMaxCtrl);                                         // Acceleration
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax *dt, desiredHeightAccelerationPreviousTick+
               jMax *dt); // Jerk

         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);

         addedHorizontalAcceleration.set(com3D);
         addedHorizontalAcceleration.sub(desiredCMPtoProject);
         addedHorizontalAcceleration.scale(yoDesiredHeightAcceleration.getDoubleValue()/com3D.getZ());
         modifiedLinearMomentumRateOfChange.set(linearMomentumRateOfChangeFromLIP.getX() + addedHorizontalAcceleration.getX()*totalMass, linearMomentumRateOfChangeFromLIP.getY() + addedHorizontalAcceleration.getY()*totalMass,yoDesiredHeightAcceleration.getDoubleValue()*totalMass);
      }
      else
      {
         FrameVector3D centerOfMassVelocity = new FrameVector3D();
         controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);
         double z = com3D.getZ();
         double dz = centerOfMassVelocity.getZ();

         /**
          * Only smoothing out differences if varying height controller kicked in
          */
         varyingHeightConditionYoEnum.set(VaryingHeightCondition.DEFAULT);
         varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.DEFAULT);
         desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax *dt, desiredHeightAccelerationPreviousTick+
               jMax *dt); // Jerk
         yoDesiredHeightAcceleration.set(desiredHeightAcceleration); // HERE DOESNT AFFECT Ld!!
         modifiedLinearMomentumRateOfChange.setIncludingFrame(linearMomentumRateOfChangeFromLIP.getReferenceFrame(),linearMomentumRateOfChangeFromLIP.getX(),linearMomentumRateOfChangeFromLIP.getY(),desiredHeightAcceleration*totalMass);
      }
   }



   private void evaluatePrimaryCondition(double z, double dz, double errorAngle, double errorAngleEndOfSwing, boolean angleGrows, VaryingHeightCondition varyingHeightConditionPreviousTick)
   {
      if((z+MathTools.sign(dz)*dz*dz/(2* -aMinPredicted)>zMax|| kneeAngle<minKneeAngle)    )                             // max height/vel and singularity
      {
         varyingHeightConditionYoEnum.set(VaryingHeightCondition.MAXZ);
      }
      else if (z+MathTools.sign(dz)*dz*dz/(2*aMaxPredicted)<zMin || kneeAngle >maxKneeAngle )                         // min height / min vel
      {
         varyingHeightConditionYoEnum.set(VaryingHeightCondition.MINZ);
      }
      else if(errorAngle<posAlignTresh && errorAngle>-posAlignTresh ) //||(!useAngleForConditions && distancePosAlignment)                                         // alignment ICPe and 'pendulum' positive force
      {
         varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_POS);
         if(varyingHeightConditionPreviousTick==VaryingHeightCondition.PREPARE_POS && angleGrows && MathTools.epsilonEquals(errorAngle,posAlignTresh,0.2))
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_POS);
         }
         else if (varyingHeightConditionPreviousTick==VaryingHeightCondition.PREPARE_POS && !angleGrows && MathTools.epsilonEquals(errorAngle,-posAlignTresh,0.2))
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_POS);
         }
      }
      else if(errorAngle>negAlignTresh|| errorAngle<-negAlignTresh )   //||(!useAngleForConditions && !distancePosAlignment)                                                         // alignment ICPe and 'pendulum' negative force
      {
         varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_NEG);
         if(varyingHeightConditionPreviousTick==VaryingHeightCondition.PREPARE_NEG && angleGrows && MathTools.epsilonEquals(errorAngle,-negAlignTresh,0.2))
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_NEG);
         }
         else if (varyingHeightConditionPreviousTick==VaryingHeightCondition.PREPARE_NEG && !angleGrows && MathTools.epsilonEquals(errorAngle,negAlignTresh,0.2))
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_NEG);
         }
      }
      else if (useAngleForConditions)      // preparing for future angle
      {
         if(errorAngleEndOfSwing<posAlignTresh && errorAngleEndOfSwing>-posAlignTresh)
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_NEG);
            if(varyingHeightConditionPreviousTick==VaryingHeightCondition.ALIGNED_POS)
            {
               varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_POS);
            }
         }
         else if (errorAngleEndOfSwing>negAlignTresh || errorAngleEndOfSwing<-negAlignTresh)
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_POS);
            if(varyingHeightConditionPreviousTick==VaryingHeightCondition.ALIGNED_NEG)
            {
               varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_NEG);
            }
         }
      }
   }

   private boolean getDistancePosAlignment(double errorAngle)
   {
      boolean distancePosAlignment;
      if(errorAngle<0.5*Math.PI && errorAngle>-0.5*Math.PI)
      {
         distancePosAlignment = true;
      }
      else
      {
         distancePosAlignment = false;
      }
      return distancePosAlignment;
   }

   private double getTMinPosReachedPredicted(double zCurrent, double dzCurrent)
   {
      double zMinForPrediction = 1.03*zMin;
      double a = 0.5*(aMinPredicted-aMinPredicted*aMinPredicted/aMaxPredicted);
      double b = dzCurrent - dzCurrent*aMinPredicted/aMaxPredicted;
      double c = zCurrent-zMinForPrediction - 0.5*dzCurrent*dzCurrent/aMaxPredicted;
      double tMinPosReachedPredicted = (-b - sqrt(b*b-4*a*c))/(2*a);
      tMinPosReachedPredicted = Math.max(0,tMinPosReachedPredicted);
      return  tMinPosReachedPredicted;
   }

   private double getTMaxPosReachedPredicted(double zCurrent, double dzCurrent)
   {
      double zMaxForPrediction= zMax;
      double a = 0.5*(aMaxPredicted+aMaxPredicted*aMaxPredicted/-aMinPredicted);
      double b = dzCurrent+dzCurrent*aMaxPredicted/-aMinPredicted;
      double c = zCurrent-zMaxForPrediction+0.5*dzCurrent*dzCurrent/-aMinPredicted;
      double tMaxPosReachedPredicted = (-b + sqrt(b*b-4*a*c))/(2*a);
      tMaxPosReachedPredicted = Math.max(0, tMaxPosReachedPredicted);
      return tMaxPosReachedPredicted;
   }

   private boolean getDistanceImproves(FrameVector2D centerOfMassVelocity2D)
   {
      double distanceImprovingAngle = icpError.angle(centerOfMassVelocity2D);
      boolean distanceImproves;
      if(distanceImprovingAngle>-0.5*Math.PI && distanceImprovingAngle<0.5*Math.PI)
      {
         distanceImproves = true;
      }
      else
      {
         distanceImproves = false;
      }
      return  distanceImproves;
   }

   private double getKneeAngle()
   {
      double angle;
      if(supportSide==null)
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

   private boolean getAngleGrows(double errorAngle, FrameVector2D centerOfMassVelocity2D)
   {
      FrameVector2D copCoMVec = new FrameVector2D();
      copCoMVec.setIncludingFrame(com3D);
      copCoMVec.sub(desiredCMPtoProject);
      double comDirectionAngle = centerOfMassVelocity2D.angle(copCoMVec);
      copCoMDirectionAngle.set(comDirectionAngle);
      boolean angleGrows = errorAngle/comDirectionAngle>0;
      if(errorAngle<0)
      {
         angleGrows = !angleGrows;
      }
      return  angleGrows;
   }

   private double getErrorAngle()
   {
      FrameVector2D copCoMVec = new FrameVector2D();
      copCoMVec.setIncludingFrame(com3D);
      copCoMVec.sub(desiredCMPtoProject);
      double errorAngle = copCoMVec.angle(icpError);
      return  errorAngle;
   }

   private boolean getAngleImproves(boolean angleGrows, double errorAngle)
   {
      boolean angleImproves;
      if(angleGrows==true && (errorAngle>-posAlignTresh && errorAngle<0 || errorAngle>negAlignTresh))
      {
         angleImproves = true;
      }
      else
      {
         angleImproves= false;
      }
      return  angleImproves;
   }

   private double getErrorAngleEndOfSwing()
   {
      double errorAngleEndOfSwing;
      FrameVector2D copCoMEndOfSwingVec = new FrameVector2D();
      copCoMEndOfSwingVec.setIncludingFrame(yoCoMEndOfSwing2DNotHacky);
      copCoMEndOfSwingVec.changeFrame(ReferenceFrame.getWorldFrame());
      copCoMEndOfSwingVec.sub(desiredCMPtoProject);
      errorAngleEndOfSwing = copCoMEndOfSwingVec.angle(icpError);
      copCoMICPeAngleFinal.set(errorAngleEndOfSwing);
      return  errorAngleEndOfSwing;
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
      this.supportSide =supportSide;
   }
   public void setIsInDoubleSupport(boolean isInDoubleSupport)
   {
      if(this.isInDoubleSupport==isInDoubleSupport)
      {
         walkingStateSwitch=false;
         this.isInDoubleSupport = isInDoubleSupport;
      }
      else
      {
         walkingStateSwitch=true;
         stateClock=0;
         this.isInDoubleSupport = isInDoubleSupport;
      }
   }
}
