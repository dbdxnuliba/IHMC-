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

   public QuadrupedStepStreamManager(YoDouble timestamp, QuadrupedReferenceFrames referenceFrames, double controlDt, QuadrupedXGaitSettings xGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.preplannedStepStream = new QuadrupedPreplannedStepStream(timestamp, registry);
      this.xGaitStepStream = new QuadrupedXGaitStepStream(referenceFrames, timestamp, controlDt, xGaitSettings, registry);

      this.stepMode.set(QuadrupedStepMode.PREPLANNED);

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return preplannedStepStream.isPlanAvailable() || xGaitStepStream.isPlanAvailable();
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

      getStepStream().onEntry();
   }

   public void doAction()
   {
      getStepStream().doAction();
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

   public void acceptTimedStepListCommand(QuadrupedTimedStepListCommand timedStepListCommand)
   {
      preplannedStepStream.accept(timedStepListCommand);
   }

   public void acceptTeleopCommand(QuadrupedTeleopCommand teleopCommand)
   {
      if(teleopCommand.isWalkingRequested())
      {
         xGaitStepStream.accept(teleopCommand);
      }
      else
      {
         xGaitStepStream.requestStop();
      }
   }

   private QuadrupedStepStream getStepStream()
   {
      return stepMode.getEnumValue() == QuadrupedStepMode.PREPLANNED ? preplannedStepStream : xGaitStepStream;
   }
}
