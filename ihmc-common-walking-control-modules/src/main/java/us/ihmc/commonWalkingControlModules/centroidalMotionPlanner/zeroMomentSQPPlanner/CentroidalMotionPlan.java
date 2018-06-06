package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface CentroidalMotionPlan
{
   void compute(double time);

   FramePoint3DReadOnly getPlannedCoMPosition();

   FrameVector3DReadOnly getPlannedCoMVelocity();

   FramePoint3DReadOnly getPlannedCoPPosition();

   FrameVector3DReadOnly getPlannedCoMAcceleration();

   FrameVector3DReadOnly getPlannedGroundReactionForce();
}
