package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedStepStreamManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedPreplannedStepStream preplannedStepStream;
   private final QuadrupedXGaitStepStream xGaitStepStream;
   private final YoEnum<QuadrupedStepMode> stepMode = new YoEnum<>("StepMode", registry, QuadrupedStepMode.class, false);
   private final YoDouble timestamp;
   private final YoBoolean isWalking = new YoBoolean("isWalking", registry);

   public QuadrupedStepStreamManager(YoDouble timestamp, QuadrupedReferenceFrames referenceFrames, double controlDt, QuadrupedXGaitSettings xGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      this.preplannedStepStream = new QuadrupedPreplannedStepStream(timestamp, registry);
      this.xGaitStepStream = new QuadrupedXGaitStepStream(referenceFrames, timestamp, controlDt, xGaitSettings, registry);

      this.stepMode.set(QuadrupedStepMode.PREPLANNED);

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return preplannedStepStream.isStepPlanAvailable() || xGaitStepStream.isStepPlanAvailable();
   }

   public void onEntry()
   {
      // default to preplanned steps if available
      if(preplannedStepStream.isStepPlanAvailable())
      {
         isWalking.set(true);
         stepMode.set(QuadrupedStepMode.PREPLANNED);
      }
      else if (xGaitStepStream.isStepPlanAvailable())
      {
         isWalking.set(true);
         stepMode.set(QuadrupedStepMode.XGAIT);
      }
      else
      {
         isWalking.set(false);
         return;
      }

      getStepStream().onEntry();
   }

   public void doAction()
   {
      if (!isWalking.getValue())
      {
         return;
      }

      getStepStream().doAction();
   }

   public void onExit()
   {
      getStepStream().onExit();
   }

   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return getStepStream().getSteps();
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      getStepStream().onLiftOff(quadrant);
   }

   public void onTouchDown(RobotQuadrant quadrant)
   {
      getStepStream().onTouchDown(quadrant);
   }

   public boolean areStepsAdjustable()
   {
      return getStepStream().areStepsAdjustable();
   }

   public void acceptTimedStepListCommand(QuadrupedTimedStepListCommand timedStepListCommand)
   {
      preplannedStepStream.acceptStepCommand(timedStepListCommand);
   }

   public void acceptTeleopCommand(QuadrupedTeleopCommand teleopCommand)
   {
      xGaitStepStream.acceptTeleopCommand(teleopCommand);

      boolean stopRequested = !teleopCommand.isWalkingRequested();
      if(stopRequested && stepMode.getEnumValue() == QuadrupedStepMode.XGAIT && isWalking.getValue())
      {
         TimeIntervalTools.removeStartTimesGreaterThan(timestamp.getDoubleValue(), getStepStream().getSteps());
         isWalking.set(false);
      }
   }

   private QuadrupedStepStream getStepStream()
   {
      return stepMode.getEnumValue() == QuadrupedStepMode.PREPLANNED ? preplannedStepStream : xGaitStepStream;
   }
}
