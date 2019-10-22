package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.List;

public class ExternalForceEstimationToolboxModule extends ToolboxModule
{
   private static final int UPDATE_PERIOD_MILLIS = 1;
   private final ExternalForceEstimationToolboxController forceEstimationToolboxController;

   public ExternalForceEstimationToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_MILLIS);
      this.forceEstimationToolboxController = new ExternalForceEstimationToolboxController(robotModel, commandInputManager, statusOutputManager, yoGraphicsListRegistry, registry);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s -> forceEstimationToolboxController.updateRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotDesiredConfigurationData.class, controllerPubGenerator, s -> forceEstimationToolboxController.updateRobotDesiredConfigurationData(s.takeNextData()));
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return forceEstimationToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      // TODO set up configuration message to set rigid body, offset, integration time, observer gain
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      // TODO create force estimation status message with solution quality
      return status;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.EXTERNAL_FORCE_ESTIMATION_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.EXTERNAL_FORCE_ESTIMATION_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
