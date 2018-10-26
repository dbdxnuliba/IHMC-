package us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public interface VaryingHeightControlModuleInterface
{
   void compute();
   FrameVector3D getModifiedLinearMomentumRateOfChange();
   void setSupportPolygon(FrameConvexPolygon2D supportPolygon);
   void setDesiredCMP(FramePoint2D desiredCMP);
   void setICPError(FrameVector2D icpError);
   void setCoM(FramePoint3D CoM);
   void setCoMEndOfSTep(FramePoint3D coMEndOfSTep);
   void setLinearMomentumRateOfChangeFromLIP(FrameVector3D linearMomentumRateOfChangeFromLIP);
   void setSupportSide(RobotSide supportSide);
   void setIsInDoubleSupport(boolean isInDoubleSupport);
   void setDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocity);
}
