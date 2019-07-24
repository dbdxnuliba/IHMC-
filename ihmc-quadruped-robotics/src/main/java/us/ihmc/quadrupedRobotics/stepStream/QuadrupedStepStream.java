package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.State;

public interface QuadrupedStepStream
{
   void onEntry();

   void doAction();

   void onExit();

   PreallocatedList<? extends QuadrupedTimedStep> getSteps();

   boolean areStepsAdjustable();

   void onTouchDown(RobotQuadrant quadrant);

   void onLiftOff(RobotQuadrant quadrant);
}
