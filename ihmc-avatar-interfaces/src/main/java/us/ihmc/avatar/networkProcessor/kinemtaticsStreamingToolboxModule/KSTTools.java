package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTTools
{
   private final CommandInputManager commandInputManager;
   private final CommandInputManager ikCommandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoVariableRegistry registry;

   private final HumanoidKinematicsToolboxController ikController;
   private final KinematicsToolboxOutputConverter outputConverter;
   private final WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   private final YoDouble streamIntegrationDuration;

   public KSTTools(CommandInputManager commandInputManager, CommandInputManager ikCommandInputManager, StatusMessageOutputManager statusOutputManager,
                   FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory,
                   YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.commandInputManager = commandInputManager;
      this.ikCommandInputManager = ikCommandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = registry;

      ikController = new HumanoidKinematicsToolboxController(ikCommandInputManager,
                                                             statusOutputManager,
                                                             desiredFullRobotModel,
                                                             yoGraphicsListRegistry,
                                                             registry);
      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      streamIntegrationDuration = new YoDouble("streamIntegrationDuration", registry);
      streamIntegrationDuration.set(0.1);
      ikController.getDefaultGains().setMaxFeedbackAndFeedbackRate(100.0, Double.POSITIVE_INFINITY);
   }

   private KinematicsStreamingToolboxInputCommand latestInput = null;

   public KinematicsStreamingToolboxInputCommand pollInputCommand()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxInputCommand.class))
         latestInput = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxInputCommand.class);
      return latestInput;
   }

   public WholeBodyTrajectoryMessage convertIKOutput()
   {
      outputConverter.updateFullRobotModel(ikController.getSolution());
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.0);

      for (RobotSide robotSide : RobotSide.values)
         outputConverter.computeArmTrajectoryMessage(robotSide);
      outputConverter.computeChestTrajectoryMessage();
      outputConverter.computePelvisTrajectoryMessage();

      HumanoidMessageTools.configureForStreaming(wholeBodyTrajectoryMessage, streamIntegrationDuration.getValue());
      return wholeBodyTrajectoryMessage;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public CommandInputManager getIKCommandInputManager()
   {
      return ikCommandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusOutputManager;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   public FullHumanoidRobotModelFactory getFullRobotModelFactory()
   {
      return fullRobotModelFactory;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public KinematicsToolboxOutputConverter getOutputConverter()
   {
      return outputConverter;
   }

   public HumanoidKinematicsToolboxController getIKController()
   {
      return ikController;
   }
}
