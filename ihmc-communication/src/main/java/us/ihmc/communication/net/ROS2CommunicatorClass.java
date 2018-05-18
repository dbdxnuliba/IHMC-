package us.ihmc.communication.net;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2ObjectPublisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.interfaces.IHMCInterfaces;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.ros2.RealtimeRos2Node;

public class ROS2CommunicatorClass<T>
{
   private final ROS2ObjectPublisher<?> publisher;
   private final T unpackingMessage;
   private final List<ObjectConsumer<T>> consumers = new ArrayList<>();
   private final SampleInfo sampleInfo = new SampleInfo();
   private static final Map<Class<?>, MutableInt> intraprocessToggle = new HashMap<>();

   @SuppressWarnings("unchecked")
   public ROS2CommunicatorClass(PubSubImplementation pubSubImplementation, RealtimeRos2Node realtimeRos2Node, Class<?> messageClass, String topicName,
                                List<GlobalObjectConsumer> globalConsumers, ROS2ObjectCommunicatorType type)
   {
      String publisherTopic = topicName;
      String subscriberTopic = topicName;

      if (type != ROS2ObjectCommunicatorType.INTRAPROCESS)
      {
         publisherTopic += type == ROS2ObjectCommunicatorType.SERVER ? "_response" : "_request";
         subscriberTopic += type == ROS2ObjectCommunicatorType.SERVER ? "_request" : "_response";
      }
      else
      {
         if (!intraprocessToggle.containsKey(messageClass))
         {
            intraprocessToggle.put(messageClass, new MutableInt());
         }
         
         boolean pseudoServer = intraprocessToggle.get(messageClass).getAndIncrement() % 2 == 0;
         publisherTopic += pseudoServer ? "_response" : "_request";
         subscriberTopic += pseudoServer ? "_request" : "_response";
      }

      publisher = ROS2Tools.createPublisher(pubSubImplementation, realtimeRos2Node, IHMCInterfaces.getPubSubType(messageClass), publisherTopic,
                                            ROS2Tools.RUNTIME_EXCEPTION);
      unpackingMessage = (T) ROS2Tools.createMessage(messageClass, ROS2Tools.RUNTIME_EXCEPTION);
      ROS2Tools.createCallbackSubscription(pubSubImplementation, realtimeRos2Node, IHMCInterfaces.getPubSubType(messageClass), subscriberTopic, subscriber -> {
         ROS2Tools.popMessage(subscriber, unpackingMessage, sampleInfo);

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
