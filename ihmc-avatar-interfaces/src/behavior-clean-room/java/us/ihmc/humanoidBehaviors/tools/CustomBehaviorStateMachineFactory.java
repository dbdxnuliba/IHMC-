package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

public class CustomBehaviorStateMachineFactory<K extends Enum<K>>
{
   private final Class<K> keyType;
   private final StateMachineFactory<K, State> factory;
   private final HashMap<K, FriendlyState> stateMap;

   /**
    * Create default name and registry. Initialize with all enum values as a FriendlyState.
    *
    * @param keyType
    */
   public CustomBehaviorStateMachineFactory(Class<K> keyType)
   {
      factory = new StateMachineFactory<>(keyType);
      this.keyType = keyType;
      String name = keyType.getSimpleName() + "Machine";
      getFactory().setNamePrefix(name).setRegistry(new YoVariableRegistry(name + "Registry"));

      stateMap = new HashMap<>();
      for (K value : EnumSet.allOf(keyType))
      {
         FriendlyState friendlyState = new FriendlyState();
         getFactory().addState(value, friendlyState);
         getStateMap().put(value, friendlyState);
      }
   }

   public void addTransition(K from, StateTransitionToAny<K> stateTransitionToAny)
   {
      for (K value : EnumSet.allOf(keyType))
      {
         factory.addTransition(from, value, timeInState-> stateTransitionToAny.shouldTransitionTo(timeInState).equals(value));
      }
   }

   public void addTransition(K from, List<K> toOptions, StateTransitionToAny<K> stateTransitionToAny)
   {
      for (K value : toOptions)
      {
         factory.addTransition(from, value, timeInState ->
         {
            K transitionTo = stateTransitionToAny.shouldTransitionTo(timeInState);

            if (!toOptions.contains(transitionTo))
            {
               throw new RuntimeException("Invalid transition to " + transitionTo + ". Options are " + toOptions);
            }

            return transitionTo.equals(value);
         });
      }
   }

   public StateMachineFactory<K, State> getFactory()
   {
      return factory;
   }

   public HashMap<K, FriendlyState> getStateMap()
   {
      return stateMap;
   }
}
