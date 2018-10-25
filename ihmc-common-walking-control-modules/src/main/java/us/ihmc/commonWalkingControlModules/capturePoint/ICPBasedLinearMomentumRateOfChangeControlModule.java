package us.ihmc.commonWalkingControlModules.capturePoint;


import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends LeggedLinearMomentumRateOfChangeControlModule
{
   private final ICPProportionalController icpProportionalController;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoBoolean desiredCMPinSafeArea;
   
   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();


   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, YoICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters)
   {
      this(referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, parentRegistry, yoGraphicsListRegistry, true, controllerToolbox, walkingControllerParameters);
   }

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
                                                          double controlDT, double totalMass, double gravityZ, YoICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
                                                          YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2DProjection, HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters)
   {
      super("", referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry, use2DProjection, controllerToolbox, walkingControllerParameters);
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.desiredCMPinSafeArea = new YoBoolean("DesiredCMPinSafeArea", registry);


      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry);


   }

   public void computeCMPInternal(FramePoint2DReadOnly desiredCMPPreviousValue)
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      desiredCMP.set(icpProportionalController.doProportionalControl(desiredCMPPreviousValue, capturePoint, desiredCapturePoint,
            finalDesiredCapturePoint, desiredCapturePointVelocity, perfectCMP, omega0));

      yoUnprojectedDesiredCMP.set(desiredCMP);

      // do projection here:
      if (!areaToProjectInto.isEmpty())
      {
         desiredCMPinSafeArea.set(safeArea.isPointInside(desiredCMP));
         if (safeArea.isPointInside(desiredCMP))
         {
            supportPolygon.setIncludingFrame(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
            areaToProjectInto.setIncludingFrame(supportPolygon);
         }

         cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, areaToProjectInto, finalDesiredCapturePoint, desiredCMP);
         if (cmpProjector.getWasCMPProjected())
            icpProportionalController.bleedOffIntegralTerm();
      }
   }

   @Override
   public void clearPlan(){}

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing){}

   @Override
   public void setFinalTransferDuration(double finalTransferDuration){}

   public void initializeForStanding(){}

   public void initializeForSingleSupport(){}

   public void initializeForTransfer(){}

   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeInSwing) {}

   @Override
   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      return false;
   }

   @Override
   public ICPOptimizationControllerInterface getICPOptimizationController()
   {
      return null;
   }

   @Override
   public void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {}

   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {}
}
