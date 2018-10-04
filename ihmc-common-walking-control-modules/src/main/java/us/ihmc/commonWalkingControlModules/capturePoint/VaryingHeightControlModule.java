package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;

public class VaryingHeightControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private enum VaryingHeightCondition {Aligned, Blah1, blah2};
   private FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private FramePoint2D desiredCMPtoProject = new FramePoint2D();
   private FrameVector2D icpError = new FrameVector2D();
   private FramePoint2D com2DtoProject = new FramePoint2D();
   private FramePoint3D com3D = new FramePoint3D();
   private FrameVector3D linearMomentumRateOfChangeFromLIP = new FrameVector3D();


   private YoFramePoint2D yoProjectedDesiredCMP;
   private YoFramePoint2D yoProjectedCoM2D;
   private final YoEnum<VaryingHeightCondition> varyingHeightConditionYoEnum;

   private double totalMass;


   private double desiredHeightAcceleration = 0;
   private YoDouble yoDesiredHeightAcceleration;


   private RobotSide supportSide = null;


   private FrameVector2D addedHorizontalAcceleration = new FrameVector2D();

   private FrameVector3D  modifiedLinearMomentumRateOfChange = new FrameVector3D();

   private HighLevelHumanoidControllerToolbox controllerToolbox;

   private OneDoFJoint kneeJoint;
   private double kneeAngle;

   private boolean isInDoubleSupport;

   private YoDouble copCoMICPeAngle;
   private YoInteger condition;

   private boolean stopBouncing;
   private boolean maxNegVelocityIsReached;
   private YoBoolean yoStay;

   public  VaryingHeightControlModule(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox=controllerToolbox;
      this.totalMass=totalMass;
      parentRegistry.addChild(registry);

      yoDesiredHeightAcceleration = new YoDouble("DesiredHeightAccelerationHeightControl", registry);
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      condition = new YoInteger("VaryingHeightCondition", registry);
      yoStay = new YoBoolean("VaryingHeightStay",registry);

      varyingHeightConditionYoEnum = new YoEnum<>("varyingHeightConditionEnum", registry, VaryingHeightCondition.class);

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

      double distance = supportPolygon.signedDistance(desiredCMPtoProject);
      boolean cmpOutsidePolygon = !supportPolygon.isPointInside(desiredCMPtoProject);
      boolean isProjected = supportPolygon.orthogonalProjection(desiredCMPtoProject);
      yoProjectedDesiredCMP.set(desiredCMPtoProject);
      yoDesiredHeightAcceleration.set(0.0);
      if(isProjected && cmpOutsidePolygon  && icpError.length()>0.025) // cmp outside polygon & icp error larger than..
      {
         FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
         yoProjectedCoM2D.set(com2DtoProject);
         FrameVector2D projectedCMPVector = new FrameVector2D(desiredCMPtoProject);
         FrameVector2D projectedCoMVector = new FrameVector2D(com2DtoProject);
         projectedCMPVector.sub(projectedCoMVector);
         double copCoMOrthogonalDistance =  com2DtoProject.distance(desiredCMPtoProject)-0.05;
         double icpXSign = MathTools.sign(icpError.getX());
         double cmpXSign = MathTools.sign(projectedCMPVector.getX());
         if(MathTools.epsilonEquals(icpXSign,cmpXSign,0.0001))
         {
            copCoMOrthogonalDistance = -com2DtoProject.distance(desiredCMPtoProject)-0.05;
         }
         FrameVector3D centerOfMassVelocity = new FrameVector3D();
         controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);

         double maxAcceleration = 5;
         double desiredJerk = 150;
         double dt = controllerToolbox.getControlDT();
         double maxCoMAccelerationFromTrajectoryController = 0.4*9.81;
         double maxHeight = 1.15;
         double z = com3D.getZ();
         double dz = centerOfMassVelocity.getZ();

         FramePoint2D com2D = new FramePoint2D();
         com2D.setIncludingFrame(com3D);
         double comDistanceToPolygon = supportPolygon.distance(com2D);





         FrameVector2D copCoMVec = new FrameVector2D();
         copCoMVec.setIncludingFrame(com3D);
         copCoMVec.sub(desiredCMPtoProject);
         double errorAngle = copCoMVec.angle(icpError);
         copCoMICPeAngle.set(errorAngle);



         if(stopBouncing || maxNegVelocityIsReached)
         {
            desiredHeightAcceleration = 0.0;
            if(z<1.15 || isInDoubleSupport==false) stopBouncing = false;
            if(dz>-0.5) maxNegVelocityIsReached = false;
            condition.set(6);
         }
         else if(z+MathTools.sign(dz)*dz*dz/(2*maxCoMAccelerationFromTrajectoryController)>maxHeight|| kneeAngle<0.25)            // max height and singularity
         {
            desiredHeightAcceleration = desiredHeightAcceleration-desiredJerk*dt;
            if (dz<0 && isInDoubleSupport==true)
            {
               stopBouncing = true;
            }
            if(dz<-0.5)
            {
               maxNegVelocityIsReached = true;
            }
            condition.set(1);
         }
         else if (com3D.getZ()<1.04 && dz<0.0)                                                                                    // min height
         {
            desiredHeightAcceleration = desiredHeightAcceleration+desiredJerk*dt;
            condition.set(2);
         }
         else if(errorAngle<0.7 && errorAngle>-0.7)                                                                               // alignment ICPe and 'pendulum' positive force
         {
            desiredHeightAcceleration = desiredHeightAcceleration +desiredJerk*dt;
            condition.set(3);
         }
         else if(errorAngle>Math.PI-1.3 && errorAngle<Math.PI+1.3)                                                                // alignment ICPe and 'pendulum' negative force
         {
            desiredHeightAcceleration = desiredHeightAcceleration - desiredJerk*dt;
            if(dz<-0.5)
            {
               maxNegVelocityIsReached = true;
            }
            condition.set(4);
         }
         else                                                                                                                     // go back to trajectory
         {
            desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
            condition.set(5);
         }



         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, -maxAcceleration,maxAcceleration);
         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);

         addedHorizontalAcceleration.set(com3D);
         addedHorizontalAcceleration.sub(desiredCMPtoProject);
         addedHorizontalAcceleration.scale(yoDesiredHeightAcceleration.getDoubleValue()/com3D.getZ());
         modifiedLinearMomentumRateOfChange.set(linearMomentumRateOfChangeFromLIP.getX() + addedHorizontalAcceleration.getX()*totalMass, linearMomentumRateOfChangeFromLIP.getY() + addedHorizontalAcceleration.getY()*totalMass,yoDesiredHeightAcceleration.getDoubleValue()*totalMass);
      }
      else
      {
         desiredHeightAcceleration = 0;
         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);
         modifiedLinearMomentumRateOfChange.setIncludingFrame(linearMomentumRateOfChangeFromLIP);
         condition.set(0);
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
      this.isInDoubleSupport = isInDoubleSupport;
   }
}
