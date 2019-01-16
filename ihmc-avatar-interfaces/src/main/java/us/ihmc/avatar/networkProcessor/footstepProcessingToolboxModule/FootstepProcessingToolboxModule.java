package us.ihmc.avatar.networkProcessor.footstepProcessingToolboxModule;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.List;

public class FootstepProcessingToolboxModule extends ToolboxModule
{
   private final FootstepProcessingToolboxController footstepProcessingToolboxController;

   public FootstepProcessingToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, PubSubImplementation pubSubImplementation) throws IOException
   {
      super(drcRobotModel.getSimpleRobotName(), drcRobotModel.createFullRobotModel(), modelProvider, false, pubSubImplementation);
      this.footstepProcessingToolboxController = new FootstepProcessingToolboxController(statusOutputManager, registry);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return footstepProcessingToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return null;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return null;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return null;
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return null;
   }
}
