package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance.VaryingHeightControlModule2;
import us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance.VaryingHeightControlModuleCentral;
import us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance.VaryingHeightControlModuleInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class LeggedLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   protected RobotSide supportSide = null;
   protected RobotSide transferToSide = null;
   protected final YoEnum<RobotSide> supportLegPreviousTick;

   private VaryingHeightControlModuleInterface varyingHeightControlModule;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private YoBoolean isInDoubleSupport;
   private FramePoint3D comEndOfStep = new FramePoint3D();


   public LeggedLinearMomentumRateOfChangeControlModule(String namePrefix, ReferenceFrames referenceFrames, double gravityZ, double totalMass,
                                                        YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                        boolean use2dProjection, HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters)
   {
      super(namePrefix, referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry, use2dProjection, controllerToolbox);

      this.controllerToolbox = controllerToolbox;

      supportLegPreviousTick = YoEnum.create(namePrefix + "SupportLegPreviousTick", "", RobotSide.class, registry, true);
      isInDoubleSupport = new YoBoolean("varyingHeightDoubleSupport",registry);


      varyingHeightControlModule = new VaryingHeightControlModuleCentral(totalMass, controllerToolbox, registry, yoGraphicsListRegistry, walkingControllerParameters);

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
   public void compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack)
   {
      super.compute(desiredCMPPreviousValue, desiredCMPToPack);
      supportLegPreviousTick.set(supportSide);
   }

   private final FramePoint3D centerOfMass = new FramePoint3D();

   @Override
   public void computeHeightModification(FrameVector3D linearMomentumRateOfChangeToModify)
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
      varyingHeightControlModule.compute();
      FrameVector3D modifiedLinearMomentumRate = new FrameVector3D();
      modifiedLinearMomentumRate.setIncludingFrame(varyingHeightControlModule.getModifiedLinearMomentumRateOfChange());
      // modifiedLinearMomentumRate.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChangeToModify.setIncludingFrame(modifiedLinearMomentumRate);
   }

   public void setFinalCoMPositionEndOfStep(FramePoint3D comEndOFStep)
   {
      this.comEndOfStep.setIncludingFrame(comEndOFStep);
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
}
