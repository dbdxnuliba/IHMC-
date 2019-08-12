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
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class KinematicsStreamingToolboxController extends ToolboxController
{
   public enum StreamingToolboxState
   {
      SLEEP, CALIBRATION, VALIDATION, STREAMING
   };

   private final KSTTools tools;
   private OutputPublisher outputPublisher = message ->
   {
   };

   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();

   private final StateMachine<StreamingToolboxState, State> stateMachine;

   private final KSTSleepState sleepState = new KSTSleepState();
   private final KSTCalibrationState calibrationState = new KSTCalibrationState();
   private final KSTValidationState validationState = new KSTValidationState();
   private final KSTStreamingState streamingState = new KSTStreamingState();

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

      stateMachine = createStateMachine();
   }

   private StateMachine<StreamingToolboxState, State> createStateMachine()
   {
      StateMachineFactory<StreamingToolboxState, State> factory = new StateMachineFactory<>(StreamingToolboxState.class);

      factory.addState(SLEEP, sleepState);
      factory.addState(CALIBRATION, calibrationState);
      factory.addState(VALIDATION, validationState);
      factory.addState(STREAMING, streamingState);

      factory.addTransition(SLEEP, CALIBRATION, timeInCurrentState -> sleepState.isDone(timeInCurrentState) && !calibrationState.isDone(timeInCurrentState));
      factory.addTransition(SLEEP, VALIDATION, timeInCurrentState -> sleepState.isDone(timeInCurrentState) && calibrationState.isDone(timeInCurrentState));

      factory.addDoneTransition(CALIBRATION, VALIDATION);

      factory.addDoneTransition(VALIDATION, STREAMING);
      factory.addTransition(VALIDATION, CALIBRATION, timeInCurrentState -> validationState.isCalibrationInvalid(timeInCurrentState));

      factory.addDoneTransition(STREAMING, SLEEP);

      return factory.build(StreamingToolboxState.SLEEP);
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      this.outputPublisher = outputPublisher;
   }

   @Override
   public boolean initialize()
   {
      return tools.getIKController().initialize();
   }

   @Override
   public void updateInternal()
   {
      stateMachine.doActionAndTransition();

      KinematicsStreamingToolboxInputCommand latestInput = tools.pollInputCommand();

      CommandInputManager ikCommandInputManager = tools.getIKCommandInputManager();

      if (latestInput != null)
      {
         if (latestInput.getControlCenterOfMass())
            ikCommandInputManager.submitCommand(latestInput.getCenterOfMassInput());
         ikCommandInputManager.submitCommands(latestInput.getRigidBodyInputs());
      }

      ikCommandInputManager.submitMessage(configurationMessage);

      tools.getIKController().updateInternal();

      outputPublisher.publish(tools.convertIKOutput());
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
