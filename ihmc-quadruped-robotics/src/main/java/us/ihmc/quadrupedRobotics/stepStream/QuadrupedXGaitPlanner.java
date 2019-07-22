package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.*;

import java.util.List;

public class QuadrupedXGaitPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FramePoint3D goalPosition = new FramePoint3D();
   private final FramePose3D xGaitRectanglePose = new FramePose3D();
   private final FramePose3D xGaitRectanglePoseAtSoS = new FramePose3D();
   private final FrameVector3D goalPositionAdjustment = new FrameVector3D();

   private final PoseReferenceFrame xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", worldFrame);
   private final QuadrantDependentList<FramePoint3D> xGaitRectangle = new QuadrantDependentList<>(() -> new FramePoint3D(xGaitRectangleFrame));
   private final EndDependentList<QuadrupedTimedStep> pastSteps = new EndDependentList<>(QuadrupedTimedStep::new);

   public void computeInitialPlan(List<? extends QuadrupedTimedStep> plannedSteps, Tuple3DReadOnly planarVelocity, RobotQuadrant initialStepQuadrant,
                                  FramePoint3D supportCentroidAtSoS, double timeAtSoS, double yawAtSoS, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }

      ReferenceFrame supportCentroidFrame = supportCentroidAtSoS.getReferenceFrame();
      supportCentroidAtSoS.changeFrame(worldFrame);
      xGaitRectanglePoseAtSoS.setPosition(supportCentroidAtSoS);
      xGaitRectanglePoseAtSoS.setOrientationYawPitchRoll(yawAtSoS, 0, 0);
      supportCentroidAtSoS.changeFrame(supportCentroidFrame);

      // plan steps
      double lastStepStartTime = timeAtSoS;
      RobotQuadrant lastStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         QuadrupedTimedStep step = plannedSteps.get(i);

         // compute step quadrant
         RobotQuadrant thisStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(thisStepQuadrant);

         // compute step timing
         double endTimeShift;
         if (i == 0)
         {
            endTimeShift = 0.0;
         }
         else
         {
            endTimeShift = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(lastStepQuadrant, xGaitSettings);
         }
         double thisStepStartTime = lastStepStartTime + endTimeShift;
         double thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();
         step.getTimeInterval().setInterval(thisStepStartTime, thisStepEndTime);

         // compute xGait rectangle pose at end of step
         extrapolatePose(xGaitRectanglePoseAtSoS, xGaitRectanglePose, planarVelocity, thisStepEndTime - timeAtSoS);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         this.goalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         step.setGoalPosition(this.goalPosition);

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update state for next step
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }

   public void computeOnlinePlan(List<? extends QuadrupedTimedStep> plannedSteps, EndDependentList<? extends QuadrupedTimedStep> latestSteps,
                                 Tuple3DReadOnly planarVelocity, double currentTime, double currentYaw, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      // initialize latest step
      QuadrupedTimedStep latestStep;
      if (latestSteps.get(RobotEnd.HIND).getTimeInterval().getEndTime() > latestSteps.get(RobotEnd.FRONT).getTimeInterval().getEndTime())
         latestStep = latestSteps.get(RobotEnd.HIND);
      else
         latestStep = latestSteps.get(RobotEnd.FRONT);

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }
      xGaitRectanglePoseAtSoS.setPosition(0, 0, 0);
      xGaitRectanglePoseAtSoS.setOrientationYawPitchRoll(currentYaw, 0, 0);

      // compute step quadrants and time intervals
      {
         RobotEnd thisStepEnd = latestStep.getRobotQuadrant().getOppositeEnd();
         pastSteps.set(RobotEnd.FRONT, latestSteps.get(RobotEnd.FRONT));
         pastSteps.set(RobotEnd.HIND, latestSteps.get(RobotEnd.HIND));

         for (int i = 0; i < plannedSteps.size(); i++)
         {
            QuadrupedTimedStep thisStep = plannedSteps.get(i);
            QuadrupedTimedStep pastStepOnSameEnd = pastSteps.get(thisStepEnd);
            QuadrupedTimedStep pastStepOnOppositeEnd = pastSteps.get(thisStepEnd.getOppositeEnd());

            thisStep.setRobotQuadrant(pastStepOnSameEnd.getRobotQuadrant().getAcrossBodyQuadrant());
            QuadrupedXGaitTools.computeStepTimeInterval(thisStep, pastStepOnSameEnd, pastStepOnOppositeEnd, xGaitSettings);
            if (currentTime > thisStep.getTimeInterval().getStartTime())
               thisStep.getTimeInterval().shiftInterval(currentTime - thisStep.getTimeInterval().getStartTime());

            pastSteps.set(thisStepEnd, thisStep);
            thisStepEnd = thisStepEnd.getOppositeEnd();
         }
      }

      // compute step goal positions and ground clearances
      {
         for (int i = 0; i < plannedSteps.size(); i++)
         {
            // compute xGait rectangle pose at end of step
            double deltaTime = plannedSteps.get(i).getTimeInterval().getEndTime() - currentTime;
            extrapolatePose(xGaitRectanglePose, xGaitRectanglePoseAtSoS, planarVelocity, deltaTime);
            xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

            // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
            RobotQuadrant stepQuadrant = plannedSteps.get(i).getRobotQuadrant();
            goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
            plannedSteps.get(i).setGoalPosition(goalPosition);

            // compute step ground clearance
            plannedSteps.get(i).setGroundClearance(xGaitSettings.getStepGroundClearance());
         }
      }

      // translate step goal positions based on latest step position
      {
         // compute xGait rectangle pose at end of step
         double deltaTime = latestStep.getTimeInterval().getEndTime() - currentTime;
         extrapolatePose(xGaitRectanglePose, xGaitRectanglePoseAtSoS, planarVelocity, deltaTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

         // compute step goal position
         RobotQuadrant stepQuadrant = latestStep.getRobotQuadrant();
         goalPositionAdjustment.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
         goalPositionAdjustment.changeFrame(worldFrame);

         // compute step goal adjustment
         FramePoint3D nominalGoalPosition = xGaitRectangle.get(stepQuadrant);
         nominalGoalPosition.changeFrame(worldFrame);
         goalPositionAdjustment.set(latestStep.getGoalPosition());
         goalPositionAdjustment.sub(nominalGoalPosition);

         // compensate for position error
         for (int i = 0; i < plannedSteps.size(); i++)
         {
            goalPosition.set(plannedSteps.get(i).getGoalPosition());
            goalPosition.changeFrame(worldFrame);
            goalPosition.add(goalPositionAdjustment);
            plannedSteps.get(i).setGoalPosition(goalPosition);
         }
      }
   }

   private void extrapolatePose(FramePose3D finalPose, FramePose3D initialPose, Tuple3DReadOnly planarVelocity, double deltaTime)
   {
      double a0 = initialPose.getYaw();
      double x0 = initialPose.getX();
      double y0 = initialPose.getY();

      // initialize forward, lateral, and rotational velocity in pose frame
      double u = planarVelocity.getX();
      double v = planarVelocity.getY();
      double phi = planarVelocity.getZ();

      // compute extrapolated pose assuming a constant planar velocity
      double a, x, y;
      double epsilon = 0.001;
      if (Math.abs(phi) > epsilon)
      {
         a = a0 + phi * deltaTime;
         x = x0 + u / phi * (Math.sin(a) - Math.sin(a0)) + v / phi * (Math.cos(a) - Math.cos(a0));
         y = y0 - u / phi * (Math.cos(a) - Math.cos(a0)) + v / phi * (Math.sin(a) - Math.sin(a0));
      }
      else
      {
         a = a0;
         x = x0 + (u * Math.cos(a) - v * Math.sin(a)) * deltaTime;
         y = y0 + (u * Math.sin(a) + v * Math.cos(a)) * deltaTime;
      }

      finalPose.setX(x);
      finalPose.setY(y);
      finalPose.setZ(initialPose.getZ());
      finalPose.setOrientationYawPitchRoll(a, initialPose.getPitch(), initialPose.getRoll());
   }
}