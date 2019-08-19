package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedPreplannedStepStream extends QuadrupedStepStream<QuadrupedTimedStepListCommand>
{
   private final DoubleProvider timestamp;

   public QuadrupedPreplannedStepStream(DoubleProvider timestamp, YoVariableRegistry parentRegistry)
   {
      super("preplanned", parentRegistry);
      this.timestamp = timestamp;
   }

   @Override
   public void onEntryInternal(QuadrupedTimedStepListCommand stepSequenceCommand, PreallocatedList<? extends QuadrupedTimedStep> stepSequence, DoubleProvider initialTransferDurationForShifting)
   {
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = stepSequenceCommand.getStepCommands();
      double timeShift = stepSequenceCommand.isExpressedInAbsoluteTime() ? 0.0 : timestamp.getValue() + initialTransferDurationForShifting.getValue();

      for (int i = 0; i < stepCommands.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.add();
         step.set(stepCommands.get(i));
         step.getTimeInterval().shiftInterval(timeShift);
      }

      stepPlanIsAdjustable.set(stepSequenceCommand.isStepPlanAdjustable());
   }

   @Override
   public void doActionInternal(QuadrupedTimedStepListCommand stepSequenceCommand, PreallocatedList<? extends QuadrupedTimedStep> stepSequence)
   {
      // Do nothing, upcoming step sequence should not be changed
   }
}
