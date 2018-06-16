package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FreeMotionControlState extends FootControlState
{
   private final FootControlMode stateEnum = FootControlMode.FREE_MOTION;

   public FreeMotionControlState(YoVariableRegistry registry)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return false;
   }

   @Override
   public void doAction(double timeInState)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void onEntry()
   {
      throw new RuntimeException("This is unacceptable");
   }

   @Override
   public void onExit()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      // TODO Auto-generated method stub
      return null;
   }
}

