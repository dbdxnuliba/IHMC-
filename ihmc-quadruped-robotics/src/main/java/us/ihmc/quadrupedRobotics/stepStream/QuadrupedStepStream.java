package us.ihmc.quadrupedRobotics.stepStream;

import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.Consumer;

/**
 * Light wrapper around step generation classes.
 * This class handles incoming commands
 */
public abstract class QuadrupedStepStream<T extends Command> implements Consumer<T>
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   /** Latest step stream command */
   private final MutableObject<T> command = new MutableObject<>();

   protected final YoBoolean stepPlanIsAdjustable;

   public QuadrupedStepStream(String namePrefix, YoVariableRegistry parentRegistry)
   {
      stepPlanIsAdjustable = new YoBoolean(namePrefix + "_stepsAreAdjustable", registry);
      parentRegistry.addChild(registry);
   }

   /**
    * Resets step stream
    */
   public void onEntry(PreallocatedList<? extends QuadrupedTimedStep> stepSequence, DoubleProvider initialTransferDurationForShifting)
   {
      T command = this.command.getValue();
      if(command == null)
      {
         throw new NullPointerException("onEntry called when command has not been received!");
      }

      onEntryInternal(command, stepSequence, initialTransferDurationForShifting);
      this.command.setValue(null);
   }

   /**
    * Initializes step sequence with the given command
    */
   abstract void onEntryInternal(T command, PreallocatedList<? extends QuadrupedTimedStep> stepSequence, DoubleProvider initialTransferDurationForShifting);

   /**
    * Updates step sequence according to given command
    */
   public void doAction(PreallocatedList<? extends QuadrupedTimedStep> stepSequence)
   {
      doActionInternal(command.getValue(), stepSequence);
      command.setValue(null);
   }

   /**
    * Updates step sequence with the given command and existing steps
    */
   abstract void doActionInternal(T command, PreallocatedList<? extends QuadrupedTimedStep> stepSequence);

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

   public boolean stepPlanIsAdjustable()
   {
      return stepPlanIsAdjustable.getValue();
   }
}
