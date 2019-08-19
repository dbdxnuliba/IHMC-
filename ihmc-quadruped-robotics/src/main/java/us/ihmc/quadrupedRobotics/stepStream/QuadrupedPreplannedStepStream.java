package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.List;

public class QuadrupedPreplannedStepStream extends QuadrupedStepStream<QuadrupedTimedStepListCommand>
{
   private final DoubleProvider timestamp;
   private final Point3D tempPoint = new Point3D();

   public QuadrupedPreplannedStepStream(DoubleProvider timestamp, FrameVector3DReadOnly upcomingStepAdjustment, YoVariableRegistry parentRegistry)
   {
      super("preplanned", upcomingStepAdjustment, parentRegistry);
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
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if (stepSequence.get(i).getTimeInterval().getStartTime() > timestamp.getValue())
         {
            tempPoint.set(stepSequence.get(i).getGoalPosition());
            tempPoint.add(upcomingStepAdjustment);
            stepSequence.get(i).setGoalPosition(tempPoint);
         }
      }
   }
}
