package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.stepStream.QuadrupedStepStreamManager;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedStepMessageHandler
{
   private static final double timeEpsilonForStepSelection = 0.05;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<RecyclingArrayDeque<SoleTrajectoryCommand>> upcomingFootTrajectoryCommandList = new QuadrantDependentList<>();

   private final YoInteger numberOfStepsToRecover = new YoInteger("numberOfStepsToRecover", registry);

   private final YoBoolean offsettingHeightPlanWithStepError = new YoBoolean("offsettingHeightPlanWithStepError", registry);
   private final DoubleParameter offsetHeightCorrectionScale = new DoubleParameter("stepHeightCorrectionErrorScaleFactor", registry, 0.25);

   private final QuadrupedStepStreamManager stepStreamManager;

   public QuadrupedStepMessageHandler(YoDouble robotTimestamp, double controlDt, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         upcomingFootTrajectoryCommandList.put(robotQuadrant, new RecyclingArrayDeque<>(SoleTrajectoryCommand.class, SoleTrajectoryCommand::set));
      }

      // the look-ahead step adjustment was doing integer division which was 1.0 for step 0 and 0.0 after, so effectively having a one step recovery
      // TODO tune this value
      numberOfStepsToRecover.set(1);

      this.stepStreamManager = new QuadrupedStepStreamManager(robotTimestamp, referenceFrames, controlDt, new QuadrupedXGaitSettings(), registry);
      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return stepStreamManager.isStepPlanAvailable();
   }

   public void handleQuadrupedTimedStepListCommand(QuadrupedTimedStepListCommand command)
   {
      if (!isValidStepPlan(command))
      {
         return;
      }

      stepStreamManager.acceptTimedStepListCommand(command);
   }

   public void acceptTeleopCommand(QuadrupedTeleopCommand command)
   {
      stepStreamManager.acceptTeleopCommand(command);
   }

   public void initialize()
   {
      stepStreamManager.onEntry();
      process();
   }

   public void process()
   {
      stepStreamManager.doAction();
   }

   private static boolean isValidStepPlan(QuadrupedTimedStepListCommand command)
   {
      double maximumStepTranslation = 1.0;
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = command.getStepCommands();

      for (int i = 0; i < command.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStepCommand stepCommand = stepCommands.get(i);
         for (int j = i + 1; j < command.getNumberOfSteps(); j++)
         {
            if (stepCommands.get(j).getRobotQuadrant() == stepCommands.get(i).getRobotQuadrant())
            {
               QuadrupedTimedStepCommand nextStepCommand = stepCommands.get(j);
               if (nextStepCommand.getGoalPosition().distance(stepCommand.getGoalPosition()) > maximumStepTranslation)
               {
                  return false;
               }
               else
               {
                  break;
               }
            }
         }
      }

      return true;
   }

   public void clearSteps()
   {
      // TODO
   }

   public void handleSoleTrajectoryCommand(List<SoleTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         SoleTrajectoryCommand command = commands.get(i);
         upcomingFootTrajectoryCommandList.get(command.getRobotQuadrant()).addLast(command);
      }
   }

   public SoleTrajectoryCommand pollFootTrajectoryForSolePositionControl(RobotQuadrant swingQuadrant)
   {
      return upcomingFootTrajectoryCommandList.get(swingQuadrant).poll();
   }

   public boolean hasFootTrajectoryForSolePositionControl(RobotQuadrant swingQuadrant)
   {
      return !upcomingFootTrajectoryCommandList.get(swingQuadrant).isEmpty();
   }

   public boolean hasFootTrajectoryForSolePositionControl()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (hasFootTrajectoryForSolePositionControl(robotQuadrant))
            return true;
      }

      return false;
   }

   public void handlePauseWalkingCommand(PauseWalkingCommand pauseWalkingCommand)
   {
   }

   public void clearUpcomingSteps()
   {
      // TODO
   }

   public void clearFootTrajectory(RobotQuadrant robotQuadrant)
   {
      upcomingFootTrajectoryCommandList.get(robotQuadrant).clear();
   }

   public void clearFootTrajectory()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         clearFootTrajectory(robotQuadrant);
   }

   public boolean isDoneWithStepSequence()
   {
      return stepStreamManager.getSteps().isEmpty();
   }

   public boolean isStepPlanAdjustable()
   {
      return true;
   }

   public void onTouchDown(RobotQuadrant robotQuadrant)
   {
      stepStreamManager.onTouchDown(robotQuadrant);
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      stepStreamManager.onLiftOff(quadrant);
   }

   private final FramePoint3D tempStep = new FramePoint3D();

   // Fixme this isn't working properly anymore
   public void shiftPlanPositionBasedOnStepAdjustment(FrameVector3DReadOnly stepAdjustment)
   {
      // TODO
//      int numberOfStepsToAdjust = Math.min(numberOfStepsToRecover.getIntegerValue(), receivedStepSequence.size());
//      for (int i = 0; i < numberOfStepsToAdjust; i++)
//      {
//         double multiplier = (numberOfStepsToRecover.getIntegerValue() - i) / (double) numberOfStepsToRecover.getIntegerValue();
//         tempStep.setIncludingFrame(receivedStepSequence.get(i).getReferenceFrame(), receivedStepSequence.get(i).getGoalPosition());
//         tempStep.scaleAdd(multiplier, stepAdjustment, tempStep);
//         receivedStepSequence.get(i).setGoalPosition(tempStep);
//      }
   }

   private final FrameVector3D stepOffsetVector = new FrameVector3D();

   public void addOffsetVectorOnTouchdown(FrameVector3DReadOnly offset)
   {
      if (!offsettingHeightPlanWithStepError.getValue())
      {
         return;
      }

      stepOffsetVector.setIncludingFrame(offset);
      stepOffsetVector.changeFrame(ReferenceFrame.getWorldFrame());

      stepOffsetVector.setX(0.0);
      stepOffsetVector.setY(0.0);
      stepOffsetVector.scale(offsetHeightCorrectionScale.getValue());

//      for (int i = 0; i < receivedStepSequence.size(); i++)
//      {
//         YoQuadrupedTimedStep step = receivedStepSequence.get(i);
//         tempStep.setIncludingFrame(step.getReferenceFrame(), step.getGoalPosition());
//         tempStep.add(stepOffsetVector);
//         step.setGoalPosition(tempStep);
//      }
   }

   public List<? extends QuadrupedTimedStep> getStepSequence()
   {
      return stepStreamManager.getSteps();
   }

   public List<? extends QuadrupedTimedStep> getActiveSteps()
   {
      return stepStreamManager.getActiveSteps();
   }

   public void reset()
   {
//      receivedStepSequence.clear();
      // TODO
   }

//   private int getIndexOfFirstStep(RobotQuadrant robotQuadrant, double timeEpsilon)
//   {
//      for (int i = 0;i  < receivedStepSequence.size(); i++)
//      {
//         QuadrupedTimedStep step = receivedStepSequence.get(i);
//
//         if (step.getRobotQuadrant() == robotQuadrant && step.getTimeInterval().epsilonContains(robotTimestamp.getDoubleValue(), timeEpsilon))
//            return i;
//      }
//
//      return -1;
//   }
}
