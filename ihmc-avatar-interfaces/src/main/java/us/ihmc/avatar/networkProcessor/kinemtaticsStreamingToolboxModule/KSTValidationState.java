package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.robotics.stateMachine.core.State;

public class KSTValidationState implements State
{
   public KSTValidationState(KSTTools tools)
   {
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onExit()
   {
   }

   public boolean isCalibrationInvalid(double timeInCurrentState)
   {
      return false;
   }
}
