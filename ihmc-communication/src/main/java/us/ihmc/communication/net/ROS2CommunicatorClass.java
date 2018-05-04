package us.ihmc.communication.net;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.ROS2ObjectPublisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.interfaces.IHMCInterfaces;
import us.ihmc.ros2.RealtimeRos2Node;

public class ROS2CommunicatorClass<T>
{
   private final ROS2ObjectPublisher<?> publisher;
   private final T unpackingMessage;
   private final List<ObjectConsumer<T>> consumers = new ArrayList<>();

   @SuppressWarnings("unchecked")
   public ROS2CommunicatorClass(RealtimeRos2Node realtimeRos2Node, Class<?> messageClass, String topicName, List<GlobalObjectConsumer> globalConsumers)
   {
      publisher = ROS2Tools.createPublisher(realtimeRos2Node, IHMCInterfaces.getPubSubType(messageClass), topicName, ROS2Tools.RUNTIME_EXCEPTION);
      unpackingMessage = (T) ROS2Tools.createMessage(messageClass, ROS2Tools.RUNTIME_EXCEPTION);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, IHMCInterfaces.getPubSubType(messageClass), topicName, subscriber -> {
         ROS2Tools.popMessage(subscriber, unpackingMessage, DefaultExceptionHandler.PRINT_STACKTRACE);
         
         System.out.println("Consuming " + unpackingMessage);
         for (int i = 0; i < consumers.size(); i++)
         {
            consumers.get(i).consumeObject(unpackingMessage);
         }
         
         for (int i = 0; i < globalConsumers.size(); i++)
         {
            globalConsumers.get(i).consumeObject(unpackingMessage);
         }
         
      }, ROS2Tools.RUNTIME_EXCEPTION);
   }

   public ROS2ObjectPublisher<?> getPublisher()
   {
      return publisher;
   }
   
   @SuppressWarnings("unchecked")
   public void addSubscriber(ObjectConsumer<?> listener)
   {
      consumers.add((ObjectConsumer<T>) listener);
   }
}
