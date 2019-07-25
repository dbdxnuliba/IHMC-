package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedPreplannedStepStream extends QuadrupedStepStream<QuadrupedTimedStepListCommand>
{
   private final DoubleProvider initialTransferDurationForShifting = new DoubleParameter("initialTransferDurationForShifting", registry, 1.0);

   public QuadrupedPreplannedStepStream(YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      super("preplanned_", timestamp);
      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntryInternal(QuadrupedTimedStepListCommand stepSequenceCommand)
   {
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = stepSequenceCommand.getStepCommands();
      double timeShift = stepSequenceCommand.isExpressedInAbsoluteTime() ? 0.0 : timestamp.getDoubleValue() + initialTransferDurationForShifting.getValue();

      for (int i = 0; i < stepCommands.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.add();
         step.set(stepCommands.get(i));
         step.getTimeInterval().shiftInterval(timeShift);
      }
   }

   @Override
   public void doActionInternal(QuadrupedTimedStepListCommand stepSequenceCommand)
   {
      // Do nothing, upcoming step sequence should not be changed
   }

   @Override
   int getPlanCapacity()
   {
      return QuadrupedStepMessageHandler.STEP_QUEUE_SIZE;
   }
}
