package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.CALIBRATION;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.SLEEP;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.STREAMING;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.VALIDATION;

import java.util.Arrays;

import org.apache.commons.lang.mutable.MutableBoolean;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsStreamingToolboxStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxConfigurationCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KinematicsStreamingToolboxController extends ToolboxController
{
   public enum KSTState
   {
      SLEEP, CALIBRATION, VALIDATION, STREAMING;

      private static final KSTState[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static KSTState fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   };

   private final KSTTools tools;

   private final YoDouble time = new YoDouble("time", registry);
   private final StateMachine<KSTState, State> stateMachine;

   private final KSTSleepState sleepState;
   private final KSTCalibrationState calibrationState;
   private final KSTValidationState validationState;
   private final KSTStreamingState streamingState;

   private final MutableBoolean isCalibrationStateRequested = new MutableBoolean(false);

   private final CommandInputManager commandInputManager;

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                               FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;

      tools = new KSTTools(commandInputManager, statusOutputManager, desiredFullRobotModel, fullRobotModelFactory, yoGraphicsListRegistry, registry);

      sleepState = new KSTSleepState(tools);
      calibrationState = new KSTCalibrationState(tools);
      validationState = new KSTValidationState(tools);
      streamingState = new KSTStreamingState(tools);

      stateMachine = createStateMachine(time);
   }

   private StateMachine<KSTState, State> createStateMachine(DoubleProvider timeProvider)
   {
      StateMachineFactory<KSTState, State> factory = new StateMachineFactory<>(KSTState.class);
      factory.setNamePrefix("mainStateMachine").setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(SLEEP, sleepState);
      factory.addState(CALIBRATION, calibrationState);
      factory.addState(VALIDATION, validationState);
      factory.addState(STREAMING, streamingState);

      //      factory.addTransition(SLEEP, CALIBRATION, timeInCurrentState -> sleepState.isDone(timeInCurrentState) && !calibrationState.isDone(timeInCurrentState));
      //      factory.addTransition(SLEEP, VALIDATION, timeInCurrentState -> sleepState.isDone(timeInCurrentState) && calibrationState.isDone(timeInCurrentState));
      //
      //      factory.addDoneTransition(CALIBRATION, VALIDATION);
      //
      //      factory.addDoneTransition(VALIDATION, STREAMING);
      //      factory.addTransition(VALIDATION, CALIBRATION, timeInCurrentState -> validationState.isCalibrationInvalid(timeInCurrentState));
      //
      //      factory.addDoneTransition(STREAMING, SLEEP);

      // TODO change transitions to SLEEP -> CALIBRATION -> VALIDATION -> STREAMING
      factory.addDoneTransition(SLEEP, CALIBRATION);
      factory.addDoneTransition(CALIBRATION, STREAMING);

      factory.addTransition(Arrays.asList(VALIDATION, STREAMING), CALIBRATION, t -> isCalibrationStateRequested.booleanValue());
      factory.addStateChangedListener((from, to) ->
      {
         if (to == CALIBRATION)
            isCalibrationStateRequested.setValue(false);
      });

      return factory.build(KSTState.SLEEP);
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      streamingState.setOutputPublisher(outputPublisher);
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   private long initialTimestamp = -1L;

   @Override
   public void updateInternal()
   {
      if (initialTimestamp == -1L)
         initialTimestamp = System.nanoTime();
      time.set(Conversions.nanosecondsToSeconds(System.nanoTime() - initialTimestamp));

      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxConfigurationCommand.class))
      {
         KinematicsStreamingToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxConfigurationCommand.class);
         if (command.getRequestCalibration())
            isCalibrationStateRequested.setValue(true);
      }

      stateMachine.doActionAndTransition();
      KinematicsStreamingToolboxStatusMessage statusMessage = new KinematicsStreamingToolboxStatusMessage();
      statusMessage.setCurrentState(stateMachine.getCurrentStateKey().toByte());
      statusOutputManager.reportStatusMessage(statusMessage);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   public static interface OutputPublisher
   {
      void publish(WholeBodyTrajectoryMessage messageToPublish);
   }

   KSTState getCurrentStateKey()
   {
      return stateMachine.getCurrentStateKey();
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      tools.updateRobotConfigurationData(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      tools.updateCapturabilityBasedStatus(newStatus);
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return tools.getDesiredFullRobotModel();
   }
}
