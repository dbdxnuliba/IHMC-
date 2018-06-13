package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states;

public class DoubleSupportMotionState extends MotionControllerState
{
   private static final MotionControllerStateEnum stateEnum = MotionControllerStateEnum.DOUBLE_SUPPORT;
   
   public DoubleSupportMotionState()
   {
      super(stateEnum);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction()
   {
      
   }

   @Override
   public void doTransitionOutOfAction()
   {
      
   }
}
