package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.stateMachine.core.State;

public abstract class FootControlState implements State
{
   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract FeedbackControlCommand<?> createFeedbackControlTemplate();
}

