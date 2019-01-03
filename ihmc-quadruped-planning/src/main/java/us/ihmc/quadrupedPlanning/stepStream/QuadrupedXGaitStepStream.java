package us.ihmc.quadrupedPlanning.stepStream;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedPlanning.*;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedPlanarBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class QuadrupedXGaitStepStream
{
   private static int NUMBER_OF_PREVIEW_STEPS = 16;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble minimumStepClearance = new YoDouble("minimumStepClearance", registry);
   private final YoDouble timestamp;

   private final YoBoolean useFancyXGait;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoFancyQuadrupedXGaitSettings fancyXGaitSettings;
   private final Vector3D desiredPlanarVelocity = new Vector3D();
   private final DoubleProvider firstStepDelay;

   private final QuadrupedXGaitPlannerInterface xGaitStepPlanner;
   private final QuadrupedXGaitPlannerInterface fancyXGaitStepPlanner;
   private final QuadrupedPlanarFootstepPlan footstepPlan;
   private final QuadrupedPlanarBodyPathProvider bodyPathProvider;

   public QuadrupedXGaitStepStream(YoBoolean useFancyXGait, YoQuadrupedXGaitSettings xGaitSettings, YoFancyQuadrupedXGaitSettings fancyXGaitSettings,
                                   YoDouble timestamp, QuadrupedPlanarBodyPathProvider bodyPathProvider, DoubleProvider firstStepDelay,
                                   YoVariableRegistry parentRegistry)
   {
      this.useFancyXGait = useFancyXGait;
      this.xGaitSettings = xGaitSettings;
      this.fancyXGaitSettings = fancyXGaitSettings;
      this.timestamp = timestamp;
      this.bodyPathProvider = bodyPathProvider;
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner(bodyPathProvider, xGaitSettings);
      this.fancyXGaitStepPlanner = new FancyQuadrupedXGaitPlanner(bodyPathProvider, fancyXGaitSettings);
      this.footstepPlan = new QuadrupedPlanarFootstepPlan(NUMBER_OF_PREVIEW_STEPS);
      this.firstStepDelay = firstStepDelay;
      minimumStepClearance.set(0.075);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   private void updateXGaitSettings()
   {
      // increase stance dimensions as a function of velocity to prevent self collisions
      double stepDuration = useFancyXGait.getValue() ? fancyXGaitSettings.getStepDuration() : xGaitSettings.getStepDuration();
      double stanceLength = useFancyXGait.getValue() ? fancyXGaitSettings.getStanceLength() : xGaitSettings.getStanceLength();
      double stanceWidth = useFancyXGait.getValue() ? fancyXGaitSettings.getStanceWidth() : xGaitSettings.getStanceWidth();

      double strideRotation = desiredPlanarVelocity.getZ() * stepDuration;
      double strideLength = Math.abs(2 * desiredPlanarVelocity.getX() * stepDuration);
      double strideWidth = Math.abs(2 * desiredPlanarVelocity.getY() * stepDuration);
      strideLength += Math.abs(stanceWidth / 2 * Math.sin(2 * strideRotation));
      strideWidth += Math.abs(stanceLength / 2 * Math.sin(2 * strideRotation));
      stanceLength = Math.max(stanceLength, strideLength / 2 + minimumStepClearance.getValue());
      stanceWidth = Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearance.getValue());

      if (useFancyXGait.getBooleanValue())
      {
         fancyXGaitSettings.setStanceLength(stanceLength);
         fancyXGaitSettings.setStanceWidth(stanceWidth);
      }
      else
      {
         xGaitSettings.setStanceLength(stanceLength);
         xGaitSettings.setStanceWidth(stanceWidth);
      }
   }

   public void onEntry()
   {
      // initialize step queue
      updateXGaitSettings();
      double initialTime = timestamp.getDoubleValue() + firstStepDelay.getValue();
      double endPhaseShift = useFancyXGait.getValue() ? fancyXGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();

      RobotQuadrant initialQuadrant = (endPhaseShift < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      bodyPathProvider.initialize();
      if (useFancyXGait.getBooleanValue())
         fancyXGaitStepPlanner.computeInitialPlan(footstepPlan, initialQuadrant, initialTime);
      else
         xGaitStepPlanner.computeInitialPlan(footstepPlan, initialQuadrant, initialTime);
      footstepPlan.initializeCurrentStepsFromPlannedSteps();
      this.process();
   }

   public void process()
   {
      double currentTime = timestamp.getDoubleValue();

      // update xgait current steps
      footstepPlan.updateCurrentSteps(timestamp.getDoubleValue());

      updateXGaitSettings();
      if (useFancyXGait.getBooleanValue())
         fancyXGaitStepPlanner.computeOnlinePlan(footstepPlan, currentTime);
      else
         xGaitStepPlanner.computeOnlinePlan(footstepPlan, currentTime);
   }

   public List<? extends QuadrupedTimedStep> getSteps()
   {
      return footstepPlan.getCompleteStepSequence(timestamp.getDoubleValue());
   }

   public QuadrupedPlanarFootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public void setStepSnapper(PointFootSnapper snapper)
   {
      xGaitStepPlanner.setStepSnapper(snapper);
      fancyXGaitStepPlanner.setStepSnapper(snapper);
   }
}
