package us.ihmc.quadrupedRobotics.stepStream;

import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;
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

   /**
    * How much to shift upcoming steps each tick. This value is set externally but is handled separately for each stream.
    * Since XGAIT does continuous replanning, this value is integrated and added after planning, while PREPLANNED does not recompute,
    * so this value is simply added to upcoming steps each tick.
    */
   protected final FrameVector3DReadOnly upcomingStepAdjustment;

   protected final YoBoolean stepPlanIsAdjustable;

   public QuadrupedStepStream(String namePrefix, FrameVector3DReadOnly upcomingStepAdjustment, YoVariableRegistry parentRegistry)
   {
      stepPlanIsAdjustable = new YoBoolean(namePrefix + "_stepsAreAdjustable", registry);
      this.upcomingStepAdjustment = upcomingStepAdjustment;
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
