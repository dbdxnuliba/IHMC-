package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Comparator;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedPreplannedStepStream implements QuadrupedStepStream
{
   private static final double delayPrecision = 1e-5;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble timestamp;
   private final DoubleProvider initialTransferDurationForShifting = new DoubleParameter("initialTransferDurationForShifting", registry, 1.0);
   private final AtomicReference<QuadrupedTimedStepListCommand> stepListCommand = new AtomicReference<>();
   private final PreallocatedList<QuadrupedTimedStep> stepList = new PreallocatedList<>(QuadrupedTimedStep.class,
                                                                                        QuadrupedTimedStep::new,
                                                                                        QuadrupedStepMessageHandler.STEP_QUEUE_SIZE);

   private final EndDependentList<QuadrupedTimedStep> activeSteps = new EndDependentList<>();

   public QuadrupedPreplannedStepStream(YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      parentRegistry.addChild(registry);
   }

   @Override
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return stepList;
   }

   @Override
   public void onEntry()
   {
      stepList.clear();
      QuadrupedTimedStepListCommand stepListCommand = this.stepListCommand.getAndSet(null);
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = stepListCommand.getStepCommands();
      double timeShift = stepListCommand.isExpressedInAbsoluteTime() ? 0.0 : timestamp.getDoubleValue() + initialTransferDurationForShifting.getValue();

      for (int i = 0; i < stepCommands.size(); i++)
      {
         QuadrupedTimedStep step = stepList.add();
         step.set(stepCommands.get(i));
         step.getTimeInterval().shiftInterval(timeShift);
      }

      stepList.sort(TimeIntervalTools.startTimeComparator);
   }

   @Override
   public void doAction()
   {
      updateActiveSteps();

      double delayTime = calculateDelayTime();
      for (int i = 0; i < stepList.size(); i++)
      {
         QuadrupedTimedStep step = stepList.get(i);
         if (activeSteps.get(step.getRobotQuadrant().getEnd()) != step)
         {
            step.getTimeInterval().shiftInterval(delayTime);
         }
      }
   }

   @Override
   public void onExit()
   {
      stepListCommand.set(null);
      stepList.clear();
   }

   private void updateActiveSteps()
   {
      activeSteps.clear();
      for (RobotEnd end : RobotEnd.values)
      {
         for (int i = 0; i < stepList.size(); i++)
         {
            QuadrupedTimedStep step = stepList.get(i);
            if (step.getRobotQuadrant().getEnd() == end && step.getTimeInterval().getStartTime() <= timestamp.getDoubleValue())
            {
               activeSteps.put(end, step);
               break;
            }
         }
      }
   }

   private double calculateDelayTime()
   {
      double delayTime = 0.0;
      for(RobotEnd end : RobotEnd.values)
      {
         if(activeSteps.get(end) == null)
            continue;

         delayTime = Math.max(delayTime, timestamp.getDoubleValue() - activeSteps.get(end).getTimeInterval().getEndTime() + delayPrecision);
      }
      return delayTime;
   }

   @Override
   public void onTouchDown(RobotQuadrant quadrant)
   {
      for (int i = 0; i < stepList.size(); i++)
      {
         if(stepList.get(i).getRobotQuadrant() == quadrant)
         {
            stepList.remove(i);
            break;
         }
      }
   }

   public boolean isStepPlanAvailable()
   {
      return stepListCommand.get() != null;
   }

   public void acceptStepCommand(QuadrupedTimedStepListCommand stepListMessage)
   {
      this.stepListCommand.set(stepListMessage);
   }
}
