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

public class FancyQuadrupedXGaitPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double maximumStepDown = 0.2;

   private final FramePoint3D goalPosition = new FramePoint3D();
   private final QuadrantDependentList<FramePoint3D> xGaitRectangle = new QuadrantDependentList<>();
   private final FramePose3D xGaitRectanglePose = new FramePose3D();
   private final PoseReferenceFrame xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", worldFrame);
   private final EndDependentList<QuadrupedTimedStep> pastSteps;

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrupedPlanarBodyPathProvider bodyPathProvider;
   private final FramePose2D bodyPathPose = new FramePose2D();
   private PointFootSnapper snapper = null;

   public FancyQuadrupedXGaitPlanner(QuadrupedPlanarBodyPathProvider bodyPathProvider, QuadrupedXGaitSettingsReadOnly xGaitSettings)
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
         plannedSteps.add();
         QuadrupedTimedOrientedStep step = plannedSteps.get(plannedSteps.size() - 1);

         // compute step quadrant
         RobotQuadrant thisStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(thisStepQuadrant);

         // compute step timing
         double thisStepStartTime;
         double thisStepEndTime;
         if (i == 0)
         {
            thisStepStartTime = timeAtStartOfStep;
            thisStepEndTime = timeAtStartOfStep + xGaitSettings.getStepDuration();
         }
         else
         {
            double endPhaseShift = thisStepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();
            double endTimeShift = xGaitSettings.getEndDoubleSupportDuration() + xGaitSettings.getStepDuration();
            endTimeShift *= Math.max(Math.min(endPhaseShift, 180.0), 0.0) / 180.0;
            thisStepStartTime = lastStepStartTime + endTimeShift;
            thisStepEndTime = optimizeStepTime(thisStepStartTime);
//            thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();
         }
         step.getTimeInterval().setStartTime(thisStepStartTime);
         step.getTimeInterval().setEndTime(thisStepEndTime);

         // compute xGait rectangle pose at end of step
         extrapolatePose(xGaitRectanglePose, thisStepEndTime);

         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         step.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         this.goalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         step.setGoalPosition(this.goalPosition);
         snapStep(step);

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update state for next step
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }

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
      // compute step quadrants and time intervals
      RobotEnd thisStepEnd = latestStep.getRobotQuadrant().getOppositeEnd();
      pastSteps.set(RobotEnd.FRONT, currentSteps.get(RobotEnd.FRONT));
      pastSteps.set(RobotEnd.HIND, currentSteps.get(RobotEnd.HIND));

      for (int i = 0; i < plannedSteps.capacity(); i++)
      {
         plannedSteps.add();
         QuadrupedTimedStep thisStep = plannedSteps.get(i);
         QuadrupedTimedStep pastStepOnSameEnd = pastSteps.get(thisStepEnd);
         QuadrupedTimedStep pastStepOnOppositeEnd = pastSteps.get(thisStepEnd.getOppositeEnd());

         thisStep.setRobotQuadrant(pastStepOnSameEnd.getRobotQuadrant().getAcrossBodyQuadrant());
         computeStepTimeInterval(thisStep, pastStepOnSameEnd, pastStepOnOppositeEnd, xGaitSettings);
         if (currentTime > thisStep.getTimeInterval().getStartTime())
            thisStep.getTimeInterval().shiftInterval(currentTime - thisStep.getTimeInterval().getStartTime());

         pastSteps.set(thisStepEnd, thisStep);
         thisStepEnd = thisStepEnd.getOppositeEnd();
      }

      // compute step goal positions and ground clearances
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         // compute xGait rectangle pose at end of step
         double time = plannedSteps.get(i).getTimeInterval().getEndTime();
         extrapolatePose(xGaitRectanglePose, time);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         plannedSteps.get(i).setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
         RobotQuadrant stepQuadrant = plannedSteps.get(i).getRobotQuadrant();
         goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
         plannedSteps.get(i).setGoalPosition(goalPosition);

         // compute step ground clearance
         plannedSteps.get(i).setGroundClearance(xGaitSettings.getStepGroundClearance());
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

   private final double stepTimeWeight = 2.0;
   private final double stepLengthWeight = 2.0;
   private final double stepYawWeight = 2.0;

   private final double maxStepTime = 1.0;
   private final double minStepTime = 0.2;
   private final double nominalLength = 0.15;


   private final double timeResolution = 0.05;

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D finalPose = new FramePose3D();

   private double optimizeStepTime(double initialTime)
   {
      double bestStepCost = Double.POSITIVE_INFINITY;
      extrapolatePose(initialPose, initialTime);

      double finalTime = initialTime + minStepTime;

      while (finalTime < initialTime + maxStepTime)
      {
         extrapolatePose(finalPose, finalTime);

         double translationCost = computeTranslationCost(initialPose, finalPose);
         double yawCost = computeYawCost(initialPose, finalPose);
         double timeCost = computeTimeCost(finalTime, initialTime);
         double stepCost = translationCost + yawCost + timeCost;

         if (stepCost < bestStepCost)
         {
            bestStepCost = stepCost;
            finalTime += timeResolution;
         }
         else
         {
            break;
         }

      }

   }

   private double computeTranslationCost(FramePose3DReadOnly initialPose, FramePose3DReadOnly finalPose)
   {
      double stepLength = initialPose.getPositionDistance(finalPose):
      double deviationFromNominal = stepLength - nominalLength;
      return stepLengthWeight * MathTools.square(deviationFromNominal);
   }

   private double computeYawCost(FramePose3DReadOnly initialPose, FramePose3DReadOnly finalPose)
   {
      double rotationDistance = initialPose.getOrientationDistance(finalPose);
      return stepYawWeight * MathTools.square(rotationDistance);
   }

   private double computeTimeCost(double finalTime, double initialTime)
   {
      double duration = finalTime - initialTime;
      double deviationFromNominal = duration - xGaitSettings.getStepDuration();
      return stepTimeWeight * MathTools.square(deviationFromNominal);
   }

   private void extrapolatePose(FixedFramePose3DBasics finalPose, double time)
   {
      bodyPathProvider.getPlanarPose(time, bodyPathPose);
      finalPose.setX(bodyPathPose.getX());
      finalPose.setY(bodyPathPose.getY());
      finalPose.setOrientationYawPitchRoll(bodyPathPose.getYaw(), finalPose.getPitch(), finalPose.getRoll());
   }

   private void computeStepTimeInterval(QuadrupedTimedStep thisStep, QuadrupedTimedStep pastStepOnSameEnd, QuadrupedTimedStep pastStepOnOppositeEnd,
                                        QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      RobotEnd thisStepEnd = thisStep.getRobotQuadrant().getEnd();
      RobotSide thisStepSide = thisStep.getRobotQuadrant().getSide();
      RobotSide pastStepSide = pastStepOnOppositeEnd.getRobotQuadrant().getSide();

      double pastStepEndTimeForSameEnd = pastStepOnSameEnd.getTimeInterval().getEndTime();
      double pastStepEndTimeForOppositeEnd = pastStepOnOppositeEnd.getTimeInterval().getEndTime();

      // Compute support durations and end phase shift.
      double nominalStepDuration = xGaitSettings.getStepDuration();
      double endDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();
      double endPhaseShift = MathTools.clamp(xGaitSettings.getEndPhaseShift(), 0, 359);
      if (thisStepEnd == RobotEnd.HIND)
         endPhaseShift = 360 - endPhaseShift;
      if (pastStepSide != thisStepSide)
         endPhaseShift = endPhaseShift - 180;

      // Compute step time interval. Step duration is scaled in the range (1.0, 1.5) to account for end phase shifts.
      double thisStepStartTime = pastStepEndTimeForSameEnd + endDoubleSupportDuration;
      double thisStepEndTime = pastStepEndTimeForOppositeEnd + (nominalStepDuration + endDoubleSupportDuration) * endPhaseShift / 180.0;
      double thisStepDuration = MathTools.clamp(thisStepEndTime - thisStepStartTime, nominalStepDuration, 1.5 * nominalStepDuration);

      thisStep.getTimeInterval().setStartTime(thisStepStartTime);
      thisStep.getTimeInterval().setEndTime(thisStepStartTime + thisStepDuration);
   }

   public void setStepSnapper(PointFootSnapper snapper)
   {
      this.snapper = snapper;
   }
}