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

public class KinematicsStreamingToolboxController extends HumanoidKinematicsToolboxController
{
   private final KinematicsToolboxOutputConverter outputConverter;
   private final WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   private final YoDouble streamIntegrationDuration = new YoDouble("streamIntegrationDuration", registry);
   private OutputPublisher outputPublisher = message ->
   {
   };

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                               FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(commandInputManager, statusOutputManager, desiredFullRobotModel, yoGraphicsListRegistry, parentRegistry);

      streamIntegrationDuration.set(0.05);
      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      this.outputPublisher = outputPublisher;
   }

   @Override
   public boolean initialize()
   {
      return super.initialize();
   }

   @Override
   public void updateInternal()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxInputCommand.class))
      {
         KinematicsStreamingToolboxInputCommand command = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxInputCommand.class);
         if (command.getControlCenterOfMass())
            commandInputManager.submitCommand(command.getCenterOfMassInput());
         commandInputManager.submitCommands(command.getRigidBodyInputs());
      }

      super.updateInternal();

      outputConverter.updateFullRobotModel(getSolution());
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.0);

      for (RobotSide robotSide : RobotSide.values)
         outputConverter.computeArmTrajectoryMessage(robotSide);
      outputConverter.computeChestTrajectoryMessage();
      outputConverter.computePelvisTrajectoryMessage();

      HumanoidMessageTools.configureForStreaming(wholeBodyTrajectoryMessage, streamIntegrationDuration.getValue());

      outputPublisher.publish(wholeBodyTrajectoryMessage);
   }

   @Override
   public boolean isDone()
   {
      return super.isDone();
   }

   public static interface OutputPublisher
   {
      void publish(WholeBodyTrajectoryMessage messageToPublish);
   }
}
