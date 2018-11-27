package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance.VaryingHeightControlModule;
import us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance.VaryingHeightControlModuleInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance.VaryingHeightPrimaryConditionEnum;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public abstract class LeggedLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   protected RobotSide supportSide = null;
   protected RobotSide transferToSide = null;
   protected final YoEnum<RobotSide> supportLegPreviousTick;

   private VaryingHeightControlModuleInterface varyingHeightControlModule;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private YoBoolean isInDoubleSupport;
   private FramePoint3D comEndOfStep = new FramePoint3D();
   private WalkingControllerParameters walkingControllerParameters;

   private YoFrameVector2D yoPreviousVertexICPd;
   private YoInteger yoPreviousVertexIndex;

   private YoDouble yoXForVec;
   private YoDouble yoYCoordinateOnVec;
   private YoDouble yoCMPY;
   private YoBoolean yoUseHeightForBalanceControl;

   public LeggedLinearMomentumRateOfChangeControlModule(String namePrefix, ReferenceFrames referenceFrames, double gravityZ, double totalMass,
                                                        YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, HighLevelHumanoidControllerToolbox controllerToolbox,
                                                        WalkingControllerParameters walkingControllerParameters)
   {
      super(namePrefix, referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry);
      this.controllerToolbox=controllerToolbox;
      this.walkingControllerParameters=walkingControllerParameters;
      supportLegPreviousTick = YoEnum.create(namePrefix + "SupportLegPreviousTick", "", RobotSide.class, registry, true);
      isInDoubleSupport = new YoBoolean("varyingHeightDoubleSupport",registry);


      yoPreviousVertexICPd = new YoFrameVector2D("previousVertexToICPD",ReferenceFrame.getWorldFrame(),registry);
      varyingHeightControlModule = new VaryingHeightControlModule(totalMass, controllerToolbox, registry, yoGraphicsListRegistry, walkingControllerParameters);

      yoXForVec = new YoDouble("xForVecHeight",registry);
      yoYCoordinateOnVec = new YoDouble("yCoordinateForVecWorld",registry);
      yoCMPY = new YoDouble("cmpyWorld",registry);
      yoPreviousVertexIndex = new YoInteger("PreviousVertexIndexHeight",registry);
      yoUseHeightForBalanceControl= new YoBoolean("UseHeightForBalanceController",registry);
      yoUseHeightForBalanceControl.set(walkingControllerParameters.useHeightForBalanceController());
   }

   public void setSupportLeg(RobotSide newSupportSide)
   {
      supportSide = newSupportSide;
      super.setSupportSide(supportSide);
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         this.transferToSide = robotSide.getOppositeSide();
   }


   @Override
   public boolean compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack)
   {
      boolean inputsAreOk = super.compute(desiredCMPPreviousValue, desiredCMPToPack);
      supportLegPreviousTick.set(supportSide);

      return inputsAreOk;
   }

   @Override
   public boolean compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack, FramePoint2D desiredCoPToPack)
   {
      boolean inputsAreOk = super.compute(desiredCMPPreviousValue, desiredCMPToPack, desiredCoPToPack);
      supportLegPreviousTick.set(supportSide);

      return inputsAreOk;
   }



   public abstract void clearPlan();

   public abstract void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   public abstract void setFinalTransferDuration(double finalTransferDuration);

   public void initializeForStanding()
   {
      isInDoubleSupport.set(true);
   }
   public void initializeForSingleSupport()
   {
      isInDoubleSupport.set(false);
   }
   public void initializeForTransfer()
   {
      isInDoubleSupport.set(true);
   }

   public abstract boolean getUpcomingFootstepSolution(Footstep footstepToPack);

   public abstract void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

   public abstract ICPOptimizationControllerInterface getICPOptimizationController();

   public abstract void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions);

   private final FramePoint3D centerOfMass = new FramePoint3D();

   @Override
   public void computeHeightModification(FrameVector3D linearMomentumRateOfChangeToModify)
   {
      if(yoUseHeightForBalanceControl.getBooleanValue())
      {
         centerOfMass.setToZero(centerOfMassFrame);

         varyingHeightControlModule.setCoM(centerOfMass);
         FrameVector2D icpError2d = new FrameVector2D();
         icpError2d.set(desiredCapturePoint);
         icpError2d.sub(capturePoint);
         varyingHeightControlModule.setICPError(icpError2d);
         varyingHeightControlModule.setDesiredCMP(desiredCMP);
         varyingHeightControlModule.setSupportPolygon(controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld());
         varyingHeightControlModule.setLinearMomentumRateOfChangeFromLIP(linearMomentumRateOfChangeToModify);
         varyingHeightControlModule.setSupportSide(supportSide);
         varyingHeightControlModule.setIsInDoubleSupport(isInDoubleSupport.getBooleanValue());
         varyingHeightControlModule.setCoMEndOfSTep(comEndOfStep);
         varyingHeightControlModule.setDesiredCapturePointVelocity(desiredCapturePointVelocity);
         varyingHeightControlModule.compute();
         FrameVector3D modifiedLinearMomentumRate = new FrameVector3D();
         modifiedLinearMomentumRate.setIncludingFrame(varyingHeightControlModule.getModifiedLinearMomentumRateOfChange());
         // modifiedLinearMomentumRate.changeFrame(centerOfMassFrame);
         linearMomentumRateOfChangeToModify.setIncludingFrame(modifiedLinearMomentumRate);
      }
   }

   @Override
   public void computeModifiedCMPHeight(FramePoint2D cmpToModify, FramePoint2DReadOnly com2D)
   {
      FrameVector2D icpError2d = new FrameVector2D();
      icpError2d.set(desiredCapturePoint);
      icpError2d.sub(capturePoint);
      if(yoUseHeightForBalanceControl.getBooleanValue())
      {
         if (varyingHeightControlModule.getPrimaryCondition() == VaryingHeightPrimaryConditionEnum.PREPARE_NEG)
         {
            FrameConvexPolygon2D supportPolygon = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();
            FramePoint2D vertex = new FramePoint2D();
            supportPolygon.getClosestVertex(com2D, vertex);
            int vertexIndex = supportPolygon.getClosestVertexIndex(com2D);
            FramePoint2DReadOnly previousVertex;
            if(vertexIndex==0)
            {
               previousVertex = supportPolygon.getVertex(3);
            }
            else if(vertexIndex==1)
            {
               previousVertex = supportPolygon.getVertex(0);
            }
            else
            {
               previousVertex = null;
            }

            if(previousVertex!=null)
            {
               yoPreviousVertexIndex.set(supportPolygon.getClosestEdgeIndex(previousVertex));
               FrameVector2D lowerFootEdgeICPd = new FrameVector2D();
               lowerFootEdgeICPd.set(desiredCapturePoint);
               lowerFootEdgeICPd.sub(previousVertex);
               yoPreviousVertexICPd.set(lowerFootEdgeICPd);
               double xForVec = capturePoint.getX()-previousVertex.getX();
               yoXForVec.set(xForVec);
               double yPreviousCoPToVec = xForVec*lowerFootEdgeICPd.getY()/lowerFootEdgeICPd.getX()+previousVertex.getY()+0.02+0.03*icpError2d.getY()/icpError2d.getX(); //
               yoYCoordinateOnVec.set(yPreviousCoPToVec);


               double midBound = (previousVertex.getY()+vertex.getY())/2;
               double cmpy = -3*(yPreviousCoPToVec-capturePoint.getY())+midBound;
               yoCMPY.set(cmpy);
               cmpy = MathTools.clamp(cmpy,midBound-0.15,midBound+0.15);
               cmpToModify.set(vertex.getX() - 0.04, cmpy);
            }
         }
      }
   }



   public void setFinalCoMPositionEndOfStep(FramePoint3D comEndOFStep)
   {
      this.comEndOfStep.setIncludingFrame(comEndOFStep);
   }
}
