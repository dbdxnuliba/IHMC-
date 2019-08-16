package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.stepStream.QuadrupedStepStreamManager;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.function.Function;

public class QuadrupedStepMessageHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<RecyclingArrayDeque<SoleTrajectoryCommand>> upcomingFootTrajectoryCommandList = new QuadrantDependentList<>();
   private final YoBoolean offsettingHeightPlanWithStepError = new YoBoolean("offsettingHeightPlanWithStepError", registry);
   private final DoubleParameter offsetHeightCorrectionScale = new DoubleParameter("stepHeightCorrectionErrorScaleFactor", registry, 0.25);

   private final QuadrupedStepStreamManager stepStreamManager;

   public QuadrupedStepMessageHandler(YoDouble robotTimestamp, double controlDt, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         upcomingFootTrajectoryCommandList.put(robotQuadrant, new RecyclingArrayDeque<>(SoleTrajectoryCommand.class, SoleTrajectoryCommand::set));
      }

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

   public void processInstantaneousStepAdjustment(Function<RobotQuadrant, FrameVector3DReadOnly> instantaneousStepAdjustment)
   {
      stepStreamManager.processInstantaneousStepAdjustment(instantaneousStepAdjustment);
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
      if (pauseWalkingCommand.isPauseRequested())
      {
         stepStreamManager.requestPause();
      }
      else
      {
         stepStreamManager.requestResume();
      }
   }

   public void abortWalking()
   {
      stepStreamManager.requestStop();
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
      return stepStreamManager.isDone();
   }

   public boolean isStepPlanAdjustable()
   {
      return stepStreamManager.stepPlanIsAdjustable();
   }

   public void onTouchDown(RobotQuadrant robotQuadrant)
   {
      stepStreamManager.onTouchDown(robotQuadrant);
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      stepStreamManager.onLiftOff(quadrant);
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

      stepStreamManager.adjustSteps(stepOffsetVector);
   }

   public List<? extends QuadrupedTimedStep> getStepSequence()
   {
      return stepStreamManager.getSteps();
   }

   public List<? extends QuadrupedTimedStep> getActiveSteps()
   {
      return stepStreamManager.getActiveSteps();
   }
}
