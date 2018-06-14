package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class FootTrajectoryParameters
{
   private final FrameVector3D defaultFinalVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -0.1);
   private final FrameVector3D defaultFinalAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);

   public double getNominalFirstSegmentPercentageDuraion()
   {
      return 0.4;
   }

   public double getNominalLastSegmentPercentageDuration()
   {
      return 0.4;
   }

   public FrameVector3DReadOnly getDefaultFinalVelocity()
   {
      return defaultFinalVelocity;
   }

   public FrameVector3DReadOnly getDefaultFinalAcceleration()
   {
      return defaultFinalAcceleration;
   }

   public double getNominalHeightAboveGround()
   {
      return 0.1;
   }
}
