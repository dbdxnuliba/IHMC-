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
import us.ihmc.yoVariables.variable.*;

import java.awt.*;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class VaryingHeightControlModule implements VaryingHeightControlModuleInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private WalkingControllerParameters walkingControllerParameters;
   private enum VaryingHeightCondition {ALIGNED_NEG, ALIGNED_POS, MAXZ, MINZ, GO_BACK, DEFAULT, PREPARE_NEG, PREPARE_POS};
   private enum VaryingHeightSecondaryCondition {HOLD, SMOOTH, DEFAULT }
   private FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private FramePoint2D desiredCMPtoProject = new FramePoint2D();
   private FramePoint2D desiredCMP = new FramePoint2D();
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

   private boolean hasSwitchInSwing;
   private boolean approachingSwich;

   private YoDouble yoTimeMinVelReached;
   private YoDouble yoTimeMaxVelReached;
   private YoDouble yoTimeMinPosReached;
   private YoDouble yoTimeMaxPosReached;
   private YoDouble yoTimeRemaining;
   private YoDouble yoTimeToOptimalAngle;

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
   double taft;
   double a;
   double b;
   double c;
   double smoothEpsilon;

   boolean walkingStateSwitch;
   private YoBoolean yoAngleGrows;

   private YoBoolean yoAngleImproves;
   private YoBoolean yoDistanceImproves;

   boolean heightControlInThisWalkingState = false;


   public  VaryingHeightControlModule(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, WalkingControllerParameters walkingControllerParameters)
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

      yoTimeMinVelReached = new YoDouble("estTimeMinVel", registry);
      yoTimeMaxVelReached = new YoDouble("estTimeMaxVel", registry);
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);
      yoTimeToOptimalAngle = new YoDouble("estTimeToOptimalAngle",registry);

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
   }

   public void compute()
   {
      stateClock=stateClock+dt;
      yoTimeInState.set(stateClock);
      VaryingHeightCondition varyingHeightConditionPreviousTick = varyingHeightConditionYoEnum.getEnumValue();
      VaryingHeightSecondaryCondition varyingHeightSecondaryConditionPreviousTick = varyingHeightSecondaryConditionYoEnum.getEnumValue();



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
      smoothEpsilon = 0.02;

      desiredHeightAccelerationPreviousTick = desiredHeightAcceleration;
      if(supportSide==null)
      {
         kneeAngle = 1.17;
      }
      else
      {
         kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(supportSide,LegJointName.KNEE_PITCH);
         kneeAngle = kneeJoint.getQ();
      }
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
      boolean isProjected = supportPolygon.orthogonalProjection(desiredCMPtoProject);

      yoProjectedDesiredCMP.set(desiredCMPtoProject);
      yoDesiredHeightAcceleration.set(0.0);


      if(isProjected && cmpOutsidePolygon  && distance>0.7*walkingControllerParameters.getMaxAllowedDistanceCMPSupport() && isInDoubleSupport==false || heightControlInThisWalkingState==true ) // cmp outside polygon & icp error larger than..
      {
         heightControlInThisWalkingState= true;
         if(walkingStateSwitch)
         {
            heightControlInThisWalkingState=false;
         }
         FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProjectEndOfSwing);
         yoProjectedCoM2D.set(com2DtoProject);

         FrameVector3D centerOfMassVelocity = new FrameVector3D();
         controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);

         double x = com3D.getX();
         double y = com3D.getY();
         double z = com3D.getZ();
         double dx = centerOfMassVelocity.getX();
         double dy = centerOfMassVelocity.getY();
         double dz = centerOfMassVelocity.getZ();
         yoCoMVelocity.set(dz);

         FrameVector2D centerOfMassVelocity2D = new FrameVector2D();
         centerOfMassVelocity2D.setIncludingFrame(centerOfMassVelocity);
         centerOfMassVelocity2D.changeFrame(ReferenceFrame.getWorldFrame());

         /**
          *  Distance current + distance direction + boolean improves
          */
         double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);
         double copCoMProjDistanceEndOFSwing = com2DtoProjectEndOfSwing.distance(desiredCMPtoProject);
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
         yoDistanceImproves.set(distanceImproves);
         /**
          * Error angle current + angle direction + boolean improves
          */
         FrameVector2D copCoMVec = new FrameVector2D();
         copCoMVec.setIncludingFrame(com3D);
         copCoMVec.sub(desiredCMPtoProject);
         double errorAngle = copCoMVec.angle(icpError);
         double comDirectionAngle = centerOfMassVelocity2D.angle(copCoMVec);
         copCoMDirectionAngle.set(comDirectionAngle);
         boolean angleGrows = errorAngle/comDirectionAngle>0;
         if(errorAngle<0)
         {
            angleGrows = !angleGrows;
         }
         yoAngleGrows.set(angleGrows);
         copCoMICPeAngle.set(errorAngle);

         boolean angleImproves;
         if(angleGrows==true && (errorAngle>-posAlignTresh && errorAngle<0 || errorAngle>negAlignTresh))
         {
            angleImproves = true;
         }
         else
         {
            angleImproves= false;
         }
         yoAngleImproves.set(angleImproves);

         /**
          * Error angle end of swing
          */
         double errorAngleEndOfSwing;
         FrameVector2D copCoMEndOfSwingVec = new FrameVector2D();
         copCoMEndOfSwingVec.setIncludingFrame(yoCoMEndOfSwing2DNotHacky);
         copCoMEndOfSwingVec.changeFrame(ReferenceFrame.getWorldFrame());
         copCoMEndOfSwingVec.sub(desiredCMPtoProject);
         errorAngleEndOfSwing = copCoMEndOfSwingVec.angle(icpError);
         copCoMICPeAngleFinal.set(errorAngleEndOfSwing);

         /**
          *
          */
         if(isInDoubleSupport)
         {
            tf = walkingControllerParameters.getDefaultTransferTime();
            tr = tf - stateClock;
            taft = 0;
            hasSwitchInSwing=false;
            approachingSwich=false;
         }
         else
         {
            tf = walkingControllerParameters.getDefaultSwingTime();
            tr = tf - stateClock;
            taft = 0;
            approachingSwich=false;
            if( MathTools.epsilonEquals(projectedCMPPolygonEdge.getFirstEndpointX(),projectedCMPPolygonEdge.getSecondEndpointX(),0.015) );
            {
               hasSwitchInSwing=true;
            }
         }
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
         else
         {
            zMax=1.18;
         }

         /**
          * Computation of time to optimal angle in trajectory (DOESN'T WORK (YET))
          */
         double alpha = (icpError.getY()*cos(-0.7)+icpError.getX()*sin(-0.7))/(icpError.getX()*cos(-0.7) -icpError.getY()*sin(-0.7));
         double omega = sqrt(9.81/1.1);
         double icpx = x + dx/omega;
         double icpy = y + dy/omega;
         a = (icpy - desiredCMP.getY()- alpha*(icpx - desiredCMP.getX()));
         b = (desiredCMP.getY()-y - alpha*(desiredCMP.getX()-x));
         c = - (icpy - desiredCMP.getY() -2*y - alpha*(icpx - desiredCMP.getX()- 2*x));
         double t1 = Math.log(2*(-b + sqrt(b*b-4*a*c))/(2*a))/omega;
         double t2 = Math.log(2*(-b - sqrt(b*b-4*a*c))/(2*a))/omega;
         double tOptimal;
         if (t1>0.0 && t1<(tf-stateClock))
         {
            tOptimal = t1;
         }
         else
         {
            tOptimal = t2;
         }
         yoTimeToOptimalAngle.set(tOptimal);

         /**
          * Computation of time to velocity constraints
          */
         double tMinVelReachedPredicted = (vMin-dz)/aMinPredicted;
         yoTimeMinVelReached.set(tMinVelReachedPredicted);
         double tMaxVelReachedPredicted = (vMax-dz)/aMaxPredicted;
         yoTimeMaxVelReached.set(tMaxVelReachedPredicted);

         /**
          * Computation of time to position constraints
          */
         a = 0.5*(aMaxPredicted+aMaxPredicted*aMaxPredicted/-aMinPredicted);
         b = dz+dz*aMaxPredicted/-aMinPredicted;
         c = z-zMax+0.5*dz*dz/-aMinPredicted;
         double tMaxPosReachedPredicted = (-b + sqrt(b*b-4*a*c))/(2*a);
         yoTimeMaxPosReached.set(tMaxPosReachedPredicted);

         a = 0.5*(aMinPredicted-aMinPredicted*aMinPredicted/aMaxPredicted);
         b = dz - dz*aMinPredicted/aMaxPredicted;
         c = z-zMin - 0.5*dz*dz/aMaxPredicted;
         double tMinPosReachedPredicted = (-b - sqrt(b*b-4*a*c))/(2*a);
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
          * Evaluate conditions
          */
         if((z+MathTools.sign(dz)*dz*dz/(2* -aMinPredicted)>zMax|| kneeAngle<minKneeAngle)    )                             // max height/vel and singularity
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.MAXZ);
            if(dz<-0.03 && tMinPred<tr && MathTools.epsilonEquals(tMinPred,tr,smoothEpsilon))
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
            }
            else if(dz<-0.03 && tMinPred<tr )
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
            }
         }
         else if (z+MathTools.sign(dz)*dz*dz/(2*aMaxPredicted)<zMin || kneeAngle >maxKneeAngle )                         // min height / min vel
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.MINZ);
            if(dz>0.03 && tMaxPred<tr && MathTools.epsilonEquals(tMaxPred,tr,smoothEpsilon))
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
            }
            else if (dz>0.03 && tMaxPred<tr )
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
            }
         }
         else if(errorAngle<posAlignTresh && errorAngle>-posAlignTresh)                                                                                         // alignment ICPe and 'pendulum' positive force
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_POS);
            if((stateClock>0.25) && tMaxPred<tr && MathTools.epsilonEquals(tMaxPred,tr,smoothEpsilon))
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
            }
            else if((stateClock>0.25) && tMaxPred<tr)
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
               else if (varyingHeightConditionPreviousTick!=VaryingHeightCondition.PREPARE_NEG && varyingHeightConditionPreviousTick!=VaryingHeightCondition.PREPARE_POS)
               {
                  if(varyingHeightSecondaryConditionPreviousTick==VaryingHeightSecondaryCondition.SMOOTH)
                  {
                     varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
                  }
               }
            }
         }
         else if(errorAngle>negAlignTresh || errorAngle<-negAlignTresh)                                                                                // alignment ICPe and 'pendulum' negative force
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_NEG);
            if((stateClock>0.25) && tMinPred<tr && MathTools.epsilonEquals(tMinPred,tr,smoothEpsilon))
            {
               varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
            }
            else if((stateClock>0.25) && tMinPred<tr)
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
               else if (varyingHeightConditionPreviousTick!=VaryingHeightCondition.PREPARE_NEG && varyingHeightConditionPreviousTick!=VaryingHeightCondition.PREPARE_POS)
               {
                  if(varyingHeightSecondaryConditionPreviousTick==VaryingHeightSecondaryCondition.SMOOTH)
                  {
                     varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
                  }
               }
            }
         }
         else if (hasSwitchInSwing)      // preparing for future angle
         {
            if(errorAngleEndOfSwing<posAlignTresh && errorAngleEndOfSwing>-posAlignTresh)
            {
               varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_NEG);
               if(varyingHeightConditionPreviousTick == VaryingHeightCondition.ALIGNED_POS && MathTools.epsilonEquals(errorAngle,posAlignTresh,0.3) && !angleGrows)
               {
                  varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_POS);
               }
               else if ((varyingHeightConditionPreviousTick == VaryingHeightCondition.ALIGNED_POS && MathTools.epsilonEquals(errorAngle,-posAlignTresh,0.3) && angleGrows))
               {
                  varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_POS);
               }
               else if(angleGrows)
               {
                  tr = (-posAlignTresh-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
               }
               else
               {
                  tr = (posAlignTresh-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
               }

               if(tr>tMinPred)
               {
                  varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
               }
               else
               {
                  varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.DEFAULT);
               }
            }
            else if (errorAngleEndOfSwing>negAlignTresh || errorAngleEndOfSwing<-negAlignTresh)
            {
               varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE_POS);

               if(varyingHeightConditionPreviousTick == VaryingHeightCondition.ALIGNED_NEG && MathTools.epsilonEquals(errorAngle,negAlignTresh,0.3) && angleGrows)
               {
                  varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_NEG);
               }
               else if ((varyingHeightConditionPreviousTick == VaryingHeightCondition.ALIGNED_NEG && MathTools.epsilonEquals(errorAngle,-negAlignTresh,0.3) && !angleGrows))
               {
                  varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_NEG);
               }
               else if (angleGrows)
               {
                  tr = (-negAlignTresh-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
               }
               else
               {
                  tr = (negAlignTresh-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
               }

               if(tr>tMaxPred)
               {
                  varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.SMOOTH);
               }
               else
               {
                  varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.DEFAULT);
               }
            }
         }
         else                                                                                                                     // go back to trajectory
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.GO_BACK);
            varyingHeightSecondaryConditionYoEnum.set(VaryingHeightSecondaryCondition.DEFAULT);
            desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
         }

         /**
          * Control
          */

         if(varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.ALIGNED_NEG || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.MAXZ || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_NEG)
         {
            if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.SMOOTH) {desiredHeightAcceleration = (tMinPred/tr)*aMinCtrl;}
            if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.HOLD) {desiredHeightAcceleration = 0;}
            if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.DEFAULT) {desiredHeightAcceleration = aMinCtrl;}
         }
         if(varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.ALIGNED_POS || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.MINZ || varyingHeightConditionYoEnum.getEnumValue()==VaryingHeightCondition.PREPARE_POS)
         {
            if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.SMOOTH) {desiredHeightAcceleration = (tMaxPred/tr)*aMaxCtrl;}
            if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.HOLD) {desiredHeightAcceleration = 0;}
            if(varyingHeightSecondaryConditionYoEnum.getEnumValue()==VaryingHeightSecondaryCondition.DEFAULT) {desiredHeightAcceleration = aMaxCtrl;}
         }


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
      this.desiredCMP.setIncludingFrame(desiredCMP);
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
