package us.ihmc.avatar.networkProcessor.footstepProcessingToolboxModule;

import controller_msgs.msg.dds.FootstepProcessingRequestMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class FootstepProcessingToolboxModule extends ToolboxModule
{
   private static final List<Class<? extends Settable<?>>> supportedStatusMessages = new ArrayList<>();
   private final FootstepProcessingToolboxController footstepProcessingToolboxController;

   static
   {
      supportedStatusMessages.add(FootstepProcessingRequestMessage.class);
   }

   public FootstepProcessingToolboxModule(DRCRobotModel drcRobotModel) throws IOException
   {
      super(drcRobotModel.getSimpleRobotName(), drcRobotModel.createFullRobotModel(), drcRobotModel.getLogModelProvider(), false, PubSubImplementation.FAST_RTPS);
      setTimeWithoutInputsBeforeGoingToSleep(1.0);
      this.footstepProcessingToolboxController = new FootstepProcessingToolboxController(drcRobotModel.getFootstepProcessingParameters(), statusOutputManager, registry);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepProcessingRequestMessage.class, getSubscriberTopicNameGenerator(),
                                           s -> footstepProcessingToolboxController.processMessage(s.takeNextData()));
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return footstepProcessingToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatusMessages;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PROCESSOR_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PROCESSOR_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
