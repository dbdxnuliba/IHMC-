package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class MotionControllerState extends FinishableState<MotionControllerStateEnum>
{
   public MotionControllerState(MotionControllerStateEnum stateEnum)
   {
      super(stateEnum);
   }
}
