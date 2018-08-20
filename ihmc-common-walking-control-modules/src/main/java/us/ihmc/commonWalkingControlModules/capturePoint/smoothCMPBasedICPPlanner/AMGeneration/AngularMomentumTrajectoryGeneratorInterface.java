package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public interface AngularMomentumTrajectoryGeneratorInterface 
{
   void clear();

   void update(double currentTime);

   void getDesiredAngularMomentum(FixedFrameVector3DBasics desiredAngMomToPack, FixedFrameVector3DBasics desiredTorqueToPack);

   void initializeForDoubleSupport(double currentTime, boolean isStanding);

   void initializeForSingleSupport(double currentTime);

   void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop);

   void computeReferenceAngularMomentumStartingFromSingleSupport(RobotSide swingFootSide);

   List<? extends AngularMomentumTrajectoryInterface> getTransferAngularMomentumTrajectories();

   List<? extends AngularMomentumTrajectoryInterface> getSwingAngularMomentumTrajectories();
}
