package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.StreamingToolboxState.CALIBRATION;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.StreamingToolboxState.SLEEP;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.StreamingToolboxState.STREAMING;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.StreamingToolboxState.VALIDATION;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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
   public enum StreamingToolboxState
   {
      SLEEP, CALIBRATION, VALIDATION, STREAMING
   };

   private final KSTTools tools;

   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();

   private final YoDouble time = new YoDouble("time", registry);
   private final StateMachine<StreamingToolboxState, State> stateMachine;

   private final KSTSleepState sleepState;
   private final KSTCalibrationState calibrationState;
   private final KSTValidationState validationState;
   private final KSTStreamingState streamingState;

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                               FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(commandInputManager, new CommandInputManager(HumanoidKinematicsToolboxController.class.getSimpleName(), KinematicsToolboxModule.supportedCommands()),
           statusOutputManager, desiredFullRobotModel, fullRobotModelFactory, yoGraphicsListRegistry, parentRegistry);
   }

   private KinematicsStreamingToolboxController(CommandInputManager commandInputManager, CommandInputManager ikCommandInputManager,
                                                StatusMessageOutputManager statusOutputManager, FullHumanoidRobotModel desiredFullRobotModel,
                                                FullHumanoidRobotModelFactory fullRobotModelFactory, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      tools = new KSTTools(commandInputManager,
                           ikCommandInputManager,
                           statusOutputManager,
                           desiredFullRobotModel,
                           fullRobotModelFactory,
                           yoGraphicsListRegistry,
                           registry);

      configurationMessage.setJointVelocityWeight(1.0);

      sleepState = new KSTSleepState(tools);
      calibrationState = new KSTCalibrationState(tools);
      validationState = new KSTValidationState(tools);
      streamingState = new KSTStreamingState(tools);

      stateMachine = createStateMachine(time);
   }

   private StateMachine<StreamingToolboxState, State> createStateMachine(DoubleProvider timeProvider)
   {
      StateMachineFactory<StreamingToolboxState, State> factory = new StateMachineFactory<>(StreamingToolboxState.class);
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
      factory.addDoneTransition(SLEEP, STREAMING);

      return factory.build(StreamingToolboxState.SLEEP);
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
      stateMachine.doActionAndTransition();
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

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      tools.getIKController().updateRobotConfigurationData(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      tools.getIKController().updateCapturabilityBasedStatus(newStatus);
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return tools.getDesiredFullRobotModel();
   }
}
