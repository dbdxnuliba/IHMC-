package us.ihmc.quadrupedPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedPlanarBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedPlanarFootstepPlan;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.time.TimeInterval;

public class FancyQuadrupedXGaitPlanner implements QuadrupedXGaitPlannerInterface
{
   private static final boolean performStepTimeOptimization = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double maximumStepDown = 0.2;

   private final FramePoint3D goalPosition = new FramePoint3D();
   private final QuadrantDependentList<FramePoint3D> xGaitRectangle = new QuadrantDependentList<>();
   private final FramePose3D xGaitRectanglePose = new FramePose3D();
   private final PoseReferenceFrame xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", worldFrame);
   private final EndDependentList<QuadrupedTimedStep> pastSteps;

   private final FancyQuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrupedPlanarBodyPathProvider bodyPathProvider;
   private final FramePose2D bodyPathPose = new FramePose2D();
   private PointFootSnapper snapper = null;

   public FancyQuadrupedXGaitPlanner(QuadrupedPlanarBodyPathProvider bodyPathProvider, FancyQuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.bodyPathProvider = bodyPathProvider;
      this.xGaitSettings = xGaitSettings;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.set(robotQuadrant, new FramePoint3D(xGaitRectangleFrame));
      }
      pastSteps = new EndDependentList<>();
      pastSteps.put(RobotEnd.FRONT, new QuadrupedTimedStep());
      pastSteps.put(RobotEnd.HIND, new QuadrupedTimedStep());
   }

   @Override
   public void computeInitialPlan(QuadrupedPlanarFootstepPlan footstepPlan, RobotQuadrant initialStepQuadrant, double timeAtStartOfStep)
   {
      bodyPathProvider.initialize();

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }

      // plan steps
      double lastStepStartTime = timeAtStartOfStep;
      RobotQuadrant lastStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
      plannedSteps.clear();
      for (int i = 0; i < plannedSteps.capacity(); i++)
      {
         QuadrupedTimedOrientedStep nextStep = plannedSteps.add();

         // compute step quadrant
         RobotQuadrant nextStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         nextStep.setRobotQuadrant(nextStepQuadrant);

         // compute step timing
         double nextStepStartTime;
         double nextStepEndTime;
         if (i == 0)
         {
            nextStepStartTime = lastStepStartTime;
            nextStepEndTime = lastStepStartTime + xGaitSettings.getStepDuration();
         }
         else
         {
            // If it's in the back, the last step was the opposite diagonal. If it's in the front, the last step was the front same side.
            double endPhaseShift = nextStepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();
            endPhaseShift = MathTools.clamp(endPhaseShift, 0.0, 180.0);

            double nominalStepDuration = xGaitSettings.getStepDuration();
            double nominalEndTimeShift;
            if (xGaitSettings.useFractionalDoubleSupport())
               nominalEndTimeShift = endPhaseShift / 180.0 * (1.0 + xGaitSettings.getDoubleSupportFraction()) * nominalStepDuration;
            else
               nominalEndTimeShift = endPhaseShift / 180.0 * (xGaitSettings.getEndDoubleSupportDuration() + nominalStepDuration);

            double stepDuration = optimizeStepDuration(lastStepStartTime + nominalEndTimeShift, nominalStepDuration);

            double endTimeShift;
            if (xGaitSettings.useFractionalDoubleSupport())
               endTimeShift = endPhaseShift / 180.0 * (1.0 + xGaitSettings.getDoubleSupportFraction()) * stepDuration;
            else
               endTimeShift = endPhaseShift / 180.0 * (xGaitSettings.getEndDoubleSupportDuration() + stepDuration);

            nextStepStartTime = lastStepStartTime + endTimeShift;
            nextStepEndTime = nextStepStartTime + stepDuration;
         }
         nextStep.getTimeInterval().setStartTime(nextStepStartTime);
         nextStep.getTimeInterval().setEndTime(nextStepEndTime);

         // compute xGait rectangle pose at end of step
         extrapolatePose(xGaitRectanglePose, nextStepEndTime);

         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         nextStep.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = nextStep.getRobotQuadrant();
         this.goalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         nextStep.setGoalPosition(this.goalPosition);
         snapStep(nextStep);

         // compute step ground clearance
         nextStep.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update state for next step
         lastStepStartTime = nextStepStartTime;
         lastStepQuadrant = nextStepQuadrant;
      }
   }

   @Override
   public void computeOnlinePlan(QuadrupedPlanarFootstepPlan footstepPlan, double currentTime)
   {
      // initialize latest step
      QuadrupedTimedStep latestStep;
      EndDependentList<QuadrupedTimedOrientedStep> currentSteps = footstepPlan.getCurrentSteps();

      if (currentSteps.get(RobotEnd.HIND).getTimeInterval().getEndTime() > currentSteps.get(RobotEnd.FRONT).getTimeInterval().getEndTime())
         latestStep = currentSteps.get(RobotEnd.HIND);
      else
         latestStep = currentSteps.get(RobotEnd.FRONT);

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }

      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
      plannedSteps.clear();

      RobotEnd thisStepEnd = latestStep.getRobotQuadrant().getOppositeEnd();
      pastSteps.set(RobotEnd.FRONT, currentSteps.get(RobotEnd.FRONT));
      pastSteps.set(RobotEnd.HIND, currentSteps.get(RobotEnd.HIND));

      // compute step goal positions and ground clearances
      for (int i = 0; i < plannedSteps.capacity(); i++)
      {
         QuadrupedTimedOrientedStep thisStep = plannedSteps.add();
         QuadrupedTimedStep pastStepOnSameEnd = pastSteps.get(thisStepEnd);
         QuadrupedTimedStep pastStepOnOppositeEnd = pastSteps.get(thisStepEnd.getOppositeEnd());

         // compute step quadrants and time intervals
         RobotQuadrant thisStepRobotQuadrant = pastStepOnSameEnd.getRobotQuadrant().getAcrossBodyQuadrant();
         RobotQuadrant pastStepRobotQuadrant = pastStepOnOppositeEnd.getRobotQuadrant();
         thisStep.setRobotQuadrant(thisStepRobotQuadrant);

         double pastStepEndTimeForSameEnd = pastStepOnSameEnd.getTimeInterval().getEndTime();
         double pastStepEndTimeForOppositeEnd = pastStepOnOppositeEnd.getTimeInterval().getEndTime();

         // Compute support durations and end phase shift.
         double nominalStepDuration = xGaitSettings.getStepDuration();
         double nominalEndDoubleSupportDuration;
         if (xGaitSettings.useFractionalDoubleSupport())
            nominalEndDoubleSupportDuration = xGaitSettings.getDoubleSupportFraction() * nominalStepDuration;
         else
            nominalEndDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();

         double endPhaseShift = MathTools.clamp(xGaitSettings.getEndPhaseShift(), 0.0, 359.0);
         if (thisStepRobotQuadrant.isQuadrantInHind())
            endPhaseShift = 360.0 - endPhaseShift;
         if (pastStepRobotQuadrant.getSide() != thisStepRobotQuadrant.getSide())
            endPhaseShift = endPhaseShift - 180.0;

         TimeInterval thisTimeInterval = thisStep.getTimeInterval();

         // Compute step time interval. Step duration is scaled in the range (1.0, 1.5) to account for end phase shifts.
         double nominalStepStartTime = pastStepEndTimeForSameEnd + nominalEndDoubleSupportDuration;
         double nominalStepEndTime = pastStepEndTimeForOppositeEnd + (nominalStepDuration + nominalEndDoubleSupportDuration) * endPhaseShift / 180.0;
         double thisStepDuration = MathTools.clamp(nominalStepEndTime - nominalStepStartTime, nominalStepDuration, 1.5 * nominalStepDuration);

         if (currentTime > thisTimeInterval.getStartTime())
            thisTimeInterval.shiftInterval(currentTime - thisTimeInterval.getStartTime());

         // optimize the step time
         double optimalStepDuration = optimizeStepDuration(nominalStepStartTime, thisStepDuration);
         double durationModification = optimalStepDuration - thisStepDuration;
         double optimalDoubleSupportDurationModifier = 0.0;
         if (xGaitSettings.useFractionalDoubleSupport() && Math.abs(durationModification) > 0.001)
            optimalDoubleSupportDurationModifier = xGaitSettings.getDoubleSupportFraction() * (optimalStepDuration - thisStepDuration);
         double initialTime = nominalStepStartTime + optimalDoubleSupportDurationModifier;
         double endTime = initialTime + optimalStepDuration;
         thisTimeInterval.setStartTime(initialTime);
         thisTimeInterval.setEndTime(endTime);

         // compute xGait rectangle pose at end of step
         extrapolatePose(xGaitRectanglePose, endTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         thisStep.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
         RobotQuadrant stepQuadrant = thisStep.getRobotQuadrant();
         goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
         thisStep.setGoalPosition(goalPosition);

         // compute step ground clearance
         thisStep.setGroundClearance(xGaitSettings.getStepGroundClearance());

         pastSteps.set(thisStepEnd, thisStep);
         thisStepEnd = thisStepEnd.getOppositeEnd();
      }

      // snap the desired footsteps to a height map, if provided
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         snapStep(plannedSteps.get(i));
      }
   }

   private void snapStep(QuadrupedTimedOrientedStep step)
   {
      snapStep(step, Double.NEGATIVE_INFINITY);
   }

   private void snapStep(QuadrupedTimedOrientedStep step, double previousStepZValue)
   {
      if (snapper != null)
      {
         goalPosition.setIncludingFrame(worldFrame, step.getGoalPosition());
         step.setGoalPosition(snapper.snapStep(goalPosition.getX(), goalPosition.getY(), previousStepZValue - maximumStepDown));
      }
   }

   private final double stepDurationWeight = 0.5;
   private final double doubleSupportDurationWeight = 1.0;
   private final double stepLengthWeight = 10.0;

   private final double maxStepDuration = 10.0;
   private final double minStepDuration = 0.15;
   private final double nominalLength = 0.15;

   private final double timeResolution = 0.025;

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D finalPose = new FramePose3D();

   private double optimizeStepDuration(double initialTime, double nominalDuration)
   {
      if (performStepTimeOptimization)
      {
         double bestStepCost = Double.POSITIVE_INFINITY;
         extrapolatePose(initialPose, initialTime);

         double duration = minStepDuration;

         while (duration < maxStepDuration)
         {
            double durationDelta = duration - nominalDuration;
            double modifiedInitialTime = initialTime;
            if (xGaitSettings.useFractionalDoubleSupport())
               modifiedInitialTime += xGaitSettings.getDoubleSupportFraction() * durationDelta;

            extrapolatePose(finalPose, modifiedInitialTime + duration);

            double translationCost = computeTranslationCost(initialPose, finalPose);
            double swingDurationCost = computeStepDurationCost(nominalDuration, duration);
            double doubleSupportDurationCost = computeDoubleSupportCost(xGaitSettings.getDoubleSupportFraction() * nominalDuration, xGaitSettings.getDoubleSupportFraction() * duration);
            double stepCost = translationCost + swingDurationCost;
            if (xGaitSettings.useFractionalDoubleSupport())
               stepCost += doubleSupportDurationCost;

            if (stepCost < bestStepCost)
            {
               bestStepCost = stepCost;
               duration += timeResolution;
            }
            else
            {
               break;
            }
         }

         return duration - timeResolution;
      }
      else
      {
         return nominalDuration;
      }
   }

   private double computeTranslationCost(FramePose3DReadOnly initialPose, FramePose3DReadOnly finalPose)
   {
      double stepLength = initialPose.getPositionDistance(finalPose);
      double deviationFromNominal = stepLength - nominalLength;
      return stepLengthWeight * MathTools.square(deviationFromNominal);
   }

   private double computeDoubleSupportCost(double nominalDuration, double currentDuration)
   {
      double deviationFromNominal = currentDuration - nominalDuration;
      return doubleSupportDurationWeight * MathTools.square(deviationFromNominal);
   }

   private double computeStepDurationCost(double nominalDuration, double currentDuration)
   {
      double deviationFromNominal = currentDuration - nominalDuration;
      return stepDurationWeight * MathTools.square(deviationFromNominal);
   }

   private void extrapolatePose(FixedFramePose3DBasics finalPose, double time)
   {
      bodyPathProvider.getPlanarPose(time, bodyPathPose);
      finalPose.setX(bodyPathPose.getX());
      finalPose.setY(bodyPathPose.getY());
      finalPose.setOrientationYawPitchRoll(bodyPathPose.getYaw(), finalPose.getPitch(), finalPose.getRoll());
   }

   @Override
   public void setStepSnapper(PointFootSnapper snapper)
   {
      this.snapper = snapper;
   }
}