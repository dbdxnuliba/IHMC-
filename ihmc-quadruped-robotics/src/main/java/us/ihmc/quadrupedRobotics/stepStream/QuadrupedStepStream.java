package us.ihmc.quadrupedRobotics.stepStream;

import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class QuadrupedStepStream<T extends Command> implements Consumer<T>
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   /** Current or most recently completed steps on each end */
   protected final EndDependentList<QuadrupedTimedStep> currentSteps = new EndDependentList<>(QuadrupedTimedStep::new);

   /** Entire step sequence, including current steps */
   protected final PreallocatedList<QuadrupedTimedStep> stepSequence = new PreallocatedList<>(QuadrupedTimedStep.class,
                                                                                              QuadrupedTimedStep::new,
                                                                                              getPlanCapacity());

   /** Flags indicating touchdown status */
   protected final QuadrantDependentList<YoBoolean> touchdownFlags = new QuadrantDependentList<>();

   /** Latest step stream command */
   private final MutableObject<T> command = new MutableObject<>();

   protected final YoDouble timestamp;

   protected final YoBoolean stopRequested;

   public QuadrupedStepStream(String namePrefix, YoDouble timestamp)
   {
      this.timestamp = timestamp;
      this.stopRequested = new YoBoolean(namePrefix + "StopRequested", registry);

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         touchdownFlags.put(quadrant, new YoBoolean(namePrefix + quadrant.getShortName() + "_touchdown", registry));
      }
   }

   /**
    * Resets step stream
    */
   public void onEntry()
   {
      stepSequence.clear();
      stopRequested.set(false);

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         touchdownFlags.get(quadrant).set(true);
      }

      T command = this.command.getValue();
      if(command == null)
      {
         throw new NullPointerException("onEntry called when command has not been received!");
      }

      onEntryInternal(command);
      this.command.setValue(null);
   }

   abstract void onEntryInternal(T t);

   public void doAction()
   {
      // Delay current steps and shift subsequent steps by maximum delay amount
      double currentStepDelay = getCurrentStepDelay();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);
         if(currentSteps.get(step.getRobotQuadrant().getEnd()) != step)
         {
            step.getTimeInterval().shiftInterval(currentStepDelay);
         }
      }

      // Remove completed steps
      TimeIntervalTools.removeEndTimesLessThan(timestamp.getDoubleValue(), stepSequence);

      // Update current steps
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);
         if(step.getTimeInterval().getStartTime() <= timestamp.getDoubleValue())
         {
            currentSteps.set(step.getRobotQuadrant().getEnd(), step);
         }
      }

      if (stopRequested.getValue())
      {
         // remove completed steps
         for (int i = stepSequence.size() - 1; i >= 0; i--)
         {
            if(touchdownFlags.get(stepSequence.get(i).getRobotQuadrant()).getValue())
            {
               stepSequence.remove(i);
            }
         }
      }
      else
      {
         doActionInternal(command.getValue());
         stepSequence.sort(TimeIntervalTools.startTimeComparator);
      }
   }

   abstract void doActionInternal(T t);

   /**
    * Shifts time interval of active steps to current include current time. Returns maximum shift value
    */
   private double getCurrentStepDelay()
   {
      double currentStepDelay = 0.0;
      for (RobotEnd end : RobotEnd.values)
      {
         QuadrupedTimedStep currentStep = currentSteps.get(end);

         boolean stepIsActive = !touchdownFlags.get(currentStep.getRobotQuadrant()).getBooleanValue();
         if (stepIsActive && currentStep.getTimeInterval().getEndTime() < timestamp.getDoubleValue())
         {
            double delay = timestamp.getDoubleValue() - currentStep.getTimeInterval().getEndTime();
            currentStep.getTimeInterval().shiftInterval(delay);
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

   public void requestStop()
   {
      stopRequested.set(true);
      TimeIntervalTools.removeStartTimesGreaterThan(timestamp.getDoubleValue(), stepSequence);
   }

   public void onTouchDown(RobotQuadrant quadrant)
   {
      touchdownFlags.get(quadrant).set(true);
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      touchdownFlags.get(quadrant).set(false);
   }

   @Override
   public void accept(T command)
   {
      this.command.setValue(command);
   }

   /**
    * Returns if an unprocessed command is available
    */
   public boolean isPlanAvailable()
   {
      return command.getValue() != null;
   }

   /**
    * Total plan capacity
    */
   abstract int getPlanCapacity();
}
