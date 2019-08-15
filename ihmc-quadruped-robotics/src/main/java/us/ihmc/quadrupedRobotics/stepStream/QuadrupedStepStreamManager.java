package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class QuadrupedStepStreamManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   static final int MAXIMUM_PLAN_CAPACITY = 200;

   private final YoDouble timestamp;

   /** Preplanned step stream generates steps from QuadrupedTimedStepListCommand */
   private final QuadrupedPreplannedStepStream preplannedStepStream;

   /** XGait step stream generates steps from QuadrupedTeleopCommand */
   private final QuadrupedXGaitStepStream xGaitStepStream;

   /** Active mode. If both commands are received, PREPLANNED is given precedence */
   private final YoEnum<QuadrupedStepMode> stepMode = new YoEnum<>("StepMode", registry, QuadrupedStepMode.class, false);

   /** Flags indicating touchdown status, set externally. True indicates a foot is in contact */
   private final QuadrantDependentList<YoBoolean> touchdownFlags = new QuadrantDependentList<>();

   /** Entire step sequence, including current steps */
   private final PreallocatedList<YoQuadrupedTimedStep> stepSequence;

   /** List of active steps. Steps get added to the list when based on start time and removed when touchdown is triggered */
   private final List<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();

   /** Transfer duration before liftoff of first step after onEntry is called */
   private final DoubleProvider initialTransferDurationForShifting = new DoubleParameter("initialTransferDurationForShifting", registry, 1.0);

   /** Holds current or next step on each robot end. Helper object for calculating step delay */
   private final EndDependentList<YoQuadrupedTimedStep> currentSteps = new EndDependentList<>();

   private final BooleanParameter delayToEnsureContactOnEachEnd = new BooleanParameter("delayToEnsureContactOnEachEnd", registry, true);

   /**
    * Variables for stopping and pausing. The expected behavior is:
    *  Stop: finishes the current step and clears any upcoming steps
    *  Pause: finishes the current step but keeps upcoming steps. When resume is requested, these steps will continue to stream
    */
   private final YoBoolean stopRequested = new YoBoolean("stopRequested", registry);
   private final YoBoolean pausedRequested = new YoBoolean("pauseRequested", registry);
   private final PreallocatedList<QuadrupedTimedStep> pausedSteps = new PreallocatedList<>(QuadrupedTimedStep.class, QuadrupedTimedStep::new, MAXIMUM_PLAN_CAPACITY);

   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedStepStreamManager(YoDouble timestamp, QuadrupedReferenceFrames referenceFrames, double controlDt, QuadrupedXGaitSettings xGaitSettings,
                                     YoVariableRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      this.preplannedStepStream = new QuadrupedPreplannedStepStream(timestamp, registry);
      this.xGaitStepStream = new QuadrupedXGaitStepStream(referenceFrames, timestamp, controlDt, xGaitSettings, registry);

      this.stepMode.set(QuadrupedStepMode.PREPLANNED);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         touchdownFlags.put(quadrant, new YoBoolean("stepStreamTouchdown_" + quadrant.getShortName(), registry));
      }

      Supplier<YoQuadrupedTimedStep> stepSupplier = SupplierBuilder.indexedSupplier(i -> new YoQuadrupedTimedStep("step" + i, registry));
      stepSequence = new PreallocatedList<>(YoQuadrupedTimedStep.class, stepSupplier, MAXIMUM_PLAN_CAPACITY);

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return preplannedStepStream.isPlanAvailable() || xGaitStepStream.isPlanAvailable() || (!pausedRequested.getValue() && !pausedSteps.isEmpty());
   }

   public void onEntry()
   {
      // default to preplanned steps if available
      if(preplannedStepStream.isPlanAvailable())
      {
         stepMode.set(QuadrupedStepMode.PREPLANNED);
      }
      else if (xGaitStepStream.isPlanAvailable())
      {
         stepMode.set(QuadrupedStepMode.XGAIT);
      }
      else
      {
         return;
      }

      // Assume all feet are in contact when onEntry is called
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         touchdownFlags.get(quadrant).set(true);
      }

      stopRequested.set(false);
      pausedRequested.set(false);
      pausedSteps.clear();

      stepSequence.clear();
      getStepStream().onEntry(stepSequence, initialTransferDurationForShifting);

      // Ensure step sequence is ordered according to start time
      stepSequence.sort(TimeIntervalTools.startTimeComparator);

      // Initialize current steps from step sequence
      for(RobotEnd end : RobotEnd.values)
      {
         currentSteps.put(end, getFirstStep(end));
      }
   }

   public void doAction()
   {
      // Delay end time of active steps if touchdown not triggered and end time has passed
      double currentStepDelay = handleActiveStepDelay();

      if(delayToEnsureContactOnEachEnd.getValue())
      {
         // Delay upcoming steps to ensure each end has contact
         for (int i = 0; i < stepSequence.size(); i++)
         {
            QuadrupedTimedStep step = stepSequence.get(i);
            if(currentSteps.get(step.getRobotQuadrant().getEnd()) != step)
            {
               step.getTimeInterval().shiftInterval(currentStepDelay);
            }
         }
      }

      // Remove completed steps
      TimeIntervalTools.removeEndTimesLessThan(timestamp.getDoubleValue(), stepSequence);

      // Update current steps
      for (RobotEnd end : RobotEnd.values)
      {
         currentSteps.put(end, getFirstStep(end));
      }

      if (!stopRequested.getValue() && !pausedRequested.getValue())
      {
         getStepStream().doAction(stepSequence);

         // Ensure step sequence is ordered according to start time
         stepSequence.sort(TimeIntervalTools.startTimeComparator);
      }

      updateActiveSteps();
   }

   /**
    * Extends time interval of active steps to current include current time. Returns maximum shift value
    */
   private double handleActiveStepDelay()
   {
      double currentStepDelay = 0.0;
      double delayEpsilon = 1e-3;

      for (RobotEnd end : RobotEnd.values)
      {
         QuadrupedTimedStep currentStep = currentSteps.get(end);
         if (currentStep == null)
         {
            continue;
         }

         boolean touchdownHasNotTriggered = !touchdownFlags.get(currentStep.getRobotQuadrant()).getBooleanValue();
         boolean endTimeHasExpired = currentStep.getTimeInterval().getEndTime() < timestamp.getDoubleValue();
         if (touchdownHasNotTriggered && endTimeHasExpired)
         {
            double delay = timestamp.getDoubleValue() - currentStep.getTimeInterval().getEndTime() + delayEpsilon;
            TimeIntervalBasics stepTimeInterval = currentStep.getTimeInterval();
            stepTimeInterval.setInterval(stepTimeInterval.getStartTime(), stepTimeInterval.getEndTime() + delay);

            currentStepDelay = Math.max(delay, currentStepDelay);
         }
      }
      return currentStepDelay;
   }

   /**
    * Returns entire step sequence, including current steps
    */
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return stepSequence;
   }

   public List<? extends QuadrupedTimedStep> getActiveSteps()
   {
      return activeSteps;
   }

   /**
    * Returns first step in stepSequence on the given end
    */
   private YoQuadrupedTimedStep getFirstStep(RobotEnd end)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if(stepSequence.get(i).getRobotQuadrant().getEnd() == end)
         {
            return stepSequence.get(i);
         }
      }

      return null;
   }

   private void updateActiveSteps()
   {
      activeSteps.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if(stepSequence.get(i).getTimeInterval().getStartTime() <= timestamp.getDoubleValue())
         {
            activeSteps.add(stepSequence.get(i));
         }
      }
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      touchdownFlags.get(quadrant).set(false);
   }

   public void onTouchDown(RobotQuadrant quadrant)
   {
      touchdownFlags.get(quadrant).set(true);
   }

   public void acceptTimedStepListCommand(QuadrupedTimedStepListCommand timedStepListCommand)
   {
      preplannedStepStream.accept(timedStepListCommand);
   }

   public void acceptTeleopCommand(QuadrupedTeleopCommand teleopCommand)
   {
      xGaitStepStream.accept(teleopCommand);
   }

   private QuadrupedStepStream getStepStream()
   {
      return stepMode.getEnumValue() == QuadrupedStepMode.PREPLANNED ? preplannedStepStream : xGaitStepStream;
   }

   public boolean stepPlanIsAdjustable()
   {
      return getStepStream().stepPlanIsAdjustable();
   }

   public void requestStop()
   {
      TimeIntervalTools.removeStartTimesGreaterThan(timestamp.getDoubleValue(), stepSequence);
      stopRequested.set(true);
   }

   public void requestPause()
   {
      if(pausedRequested.getValue())
      {
         return;
      }

      pausedSteps.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if(stepSequence.get(i).getTimeInterval().getStartTime() > timestamp.getDoubleValue())
         {
            pausedSteps.add().set(stepSequence.get(i));
         }
      }

      TimeIntervalTools.removeStartTimesGreaterThan(timestamp.getDoubleValue(), stepSequence);
      pausedRequested.set(true);
   }

   public void requestResume()
   {
      pausedRequested.set(false);
      double timeShift = timestamp.getDoubleValue() + initialTransferDurationForShifting.getValue() - pausedSteps.get(0).getTimeInterval().getStartTime();

      for (int i = 0; i < pausedSteps.size(); i++)
      {
         pausedSteps.get(i).getTimeInterval().shiftInterval(timeShift);
         stepSequence.add().set(pausedSteps.get(i));
      }
   }

   public boolean isDone()
   {
      return stepSequence.isEmpty();
   }

   /**
    * Manually adjust all steps by the given offset vector
    */
   public void adjustSteps(FrameVector3DReadOnly adjustmentVector)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         YoQuadrupedTimedStep step = stepSequence.get(i);
         tempPoint.setIncludingFrame(step.getReferenceFrame(), step.getGoalPosition());
         tempPoint.add(adjustmentVector);
         step.setGoalPosition(tempPoint);
      }
   }
}
