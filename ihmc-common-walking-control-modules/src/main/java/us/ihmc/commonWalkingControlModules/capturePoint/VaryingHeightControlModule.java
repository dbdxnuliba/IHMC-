package us.ihmc.commonWalkingControlModules.capturePoint;

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

public class VaryingHeightControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private enum VaryingHeightCondition {ALIGNED_NEG, ALIGNED_POS, MAXZ, MINZ, HOLD, GO_BACK, DEFAULT, PREPARE};
   private FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private FramePoint2D desiredCMPtoProject = new FramePoint2D();
   private FramePoint2D desiredCMP = new FramePoint2D();
   private FrameVector2D icpError = new FrameVector2D();
   private FramePoint2D com2DtoProject = new FramePoint2D();
   private FramePoint3D com3D = new FramePoint3D();
   private FrameVector3D linearMomentumRateOfChangeFromLIP = new FrameVector3D();
   private FramePoint2D comEndOfStep2D = new FramePoint2D();
   private YoFramePoint2D yoCoMEndOfSTep2D;


   private YoFramePoint2D yoProjectedDesiredCMP;
   private YoFramePoint2D yoProjectedCoM2D;
   private final YoEnum<VaryingHeightCondition> varyingHeightConditionYoEnum;

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

   private boolean hasSwitchInSwing;
   private boolean approachingSwich;

   private YoDouble yoTimeMinVelReached;
   private YoDouble yoTimeMaxVelReached;
   private YoDouble yoTimeMinPosReached;
   private YoDouble yoTimeMaxPosReached;
   private YoDouble yoTimeRemaining;
   private YoDouble yoTimeToOptimalAngle;

   private YoDouble yoCoMVelocity;

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

   boolean walkingStateSwitch;
   int ssCounter=0;

   public  VaryingHeightControlModule(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox=controllerToolbox;
      this.totalMass=totalMass;
      parentRegistry.addChild(registry);
      dt = controllerToolbox.getControlDT();
      yoDesiredHeightAcceleration = new YoDouble("DesiredHeightAccelerationHeightControl", registry);
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      yoCoMEndOfSTep2D = new YoFramePoint2D("varyingHeightCoMEndOfSTep",ReferenceFrame.getWorldFrame(), registry);

      yoTimeMinVelReached = new YoDouble("estTimeMinVel", registry);
      yoTimeMaxVelReached = new YoDouble("estTimeMaxVel", registry);
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);
      yoTimeToOptimalAngle = new YoDouble("estTimeToOptimalAngle",registry);

      yoCoMVelocity = new YoDouble("comVelocityHeightControl", registry);

      varyingHeightConditionYoEnum = new YoEnum<>("varyingHeightConditionEnum", registry, VaryingHeightCondition.class);
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


      if(isProjected && cmpOutsidePolygon  && distance>0.04 && isInDoubleSupport==false) // cmp outside polygon & icp error larger than..
      {

         FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
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

         FrameVector2D copCoMVec = new FrameVector2D();
         copCoMVec.setIncludingFrame(com3D);
         copCoMVec.sub(desiredCMPtoProject);
         double errorAngle = copCoMVec.angle(icpError);
         copCoMICPeAngle.set(errorAngle);

         // Very hacky, fix later (this is only for 4th and 5th step)
         double errorAngleEndOfSwing;
         FrameVector2D copCoMEndOfSwingVec = new FrameVector2D();
         comEndOfStep2D.set(0.0,0.0);
         if(com3D.getX()>1.25 && com3D.getX()<1.68)
         {
            comEndOfStep2D.set(1.68,-0.028);
         }
         else if(com3D.getX()>1.75 && com3D.getX()<2.178)
         {
            comEndOfStep2D.set(2.178,0.021);
         }
         copCoMEndOfSwingVec.setIncludingFrame(comEndOfStep2D);
         copCoMEndOfSwingVec.changeFrame(ReferenceFrame.getWorldFrame());
         copCoMEndOfSwingVec.sub(desiredCMPtoProject);
         errorAngleEndOfSwing = copCoMEndOfSwingVec.angle(icpError);

         double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);

         if(isInDoubleSupport)
         {
            tf = 0.15;
            tr = tf - stateClock;
            taft = 0;
            hasSwitchInSwing=false;
            approachingSwich=false;
         }
         else
         {
            tf = 0.6;
            tr = tf - stateClock;
            taft = 0;
            approachingSwich=false;
            if( MathTools.epsilonEquals(projectedCMPPolygonEdge.getFirstEndpointX(),projectedCMPPolygonEdge.getSecondEndpointX(),0.04) );
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
            zMax=1.17;
         }

         /**
          * Computation of time to optimal angle in trajectory (DOESN'T WORK (YET))
          */
         double alpha = (icpError.getY()*cos(-0.7)+icpError.getX()*sin(-0.7))/(icpError.getX()*cos(-0.7) -icpError.getY()*sin(-0.7));
         double halveEomegaT;
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

         /**
          * Conditions
          */
         if(z+MathTools.sign(dz)*dz*dz/(2* -aMinPredicted)>zMax|| kneeAngle<minKneeAngle )                             // max height/vel and singularity
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.MAXZ);
            desiredHeightAcceleration = desiredHeightAccelerationPreviousTick- jMax *dt;
            if(dz<-0.03 && tMinVelReachedPredicted<tr )
            {
               desiredHeightAcceleration=0.0;
            }
         }
         else if (z+MathTools.sign(dz)*dz*dz/(2*aMaxPredicted)<zMin || kneeAngle >maxKneeAngle )                         // min height / min vel
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.MINZ);
            desiredHeightAcceleration = desiredHeightAccelerationPreviousTick+ jMax *dt;
            if (dz>0.03 && tMaxPosReachedPredicted<tr )
            {
               desiredHeightAcceleration=0.0;
            }
         }
         else if(errorAngle<0.7 && errorAngle>-0.7)                                                                                         // alignment ICPe and 'pendulum' positive force
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_POS);
            desiredHeightAcceleration = desiredHeightAccelerationPreviousTick + jMax *dt;
            if((stateClock>0.2 && stateClock<0.6) && (tMaxVelReachedPredicted<tr || tMaxPosReachedPredicted<tr))
            {
               desiredHeightAcceleration = 0.0;
            }
         }
         else if(errorAngle>Math.PI-1.3 || errorAngle<-Math.PI+1.3)                                                                                // alignment ICPe and 'pendulum' negative force
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.ALIGNED_NEG);
            desiredHeightAcceleration = desiredHeightAccelerationPreviousTick - jMax *dt;
            if((stateClock>0.25 && stateClock<0.6) && (tMinVelReachedPredicted<tr))
            {
               desiredHeightAcceleration = 0.0;
            }
         }
         else if (hasSwitchInSwing)      // preparing for future angle
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.PREPARE);
            if(errorAngleEndOfSwing<0.7)
            {
               desiredHeightAcceleration = desiredHeightAccelerationPreviousTick - jMax * dt;
               double tbreak = (0.7-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
               if(tbreak>tMinPosReachedPredicted)
               {
                  aMinCtrl = (tMinPosReachedPredicted/tbreak)*aMinCtrl;
               }
            }
            else if (errorAngleEndOfSwing>Math.PI-1.3)
            {
               desiredHeightAcceleration = desiredHeightAccelerationPreviousTick + jMax * dt;
               double tbreak = (Math.PI-1.3-errorAngle)/(errorAngleEndOfSwing-errorAngle)*tr;
               if(tbreak>tMaxPosReachedPredicted)
               {
                  aMaxCtrl = (tMinPosReachedPredicted/tbreak)*aMaxCtrl;
               }
            }

         }
         else                                                                                                                     // go back to trajectory
         {
            varyingHeightConditionYoEnum.set(VaryingHeightCondition.GO_BACK);
            desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
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
         if(isInDoubleSupport=false)ssCounter=ssCounter+1;
         this.isInDoubleSupport = isInDoubleSupport;
      }
   }
}
