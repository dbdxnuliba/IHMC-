package us.ihmc.commonWalkingControlModules.controlModules.capturePointCenterOfPressure;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DoNothingCapturePointCenterOfPressureControlModule implements CapturePointCenterOfPressureControlModule
{
   private ReferenceFrame midFeetZUpFrame;

   public DoNothingCapturePointCenterOfPressureControlModule(ReferenceFrame midFeetZUpFrame)
   {
      this.midFeetZUpFrame = midFeetZUpFrame;
   }
   
   public void controlDoubleSupport(BipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
                                    FramePoint centerOfMassPositionInWorldFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
   }

   public void controlSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, RobotSide supportLeg,
                                    ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons, FramePoint centerOfMassPositionInZUpFrame,
                                    FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
   }

   public void packDesiredCenterOfPressure(FramePoint desiredCenterOfPressureToPack)
   {
      desiredCenterOfPressureToPack.setToZero(midFeetZUpFrame);
   }

}
