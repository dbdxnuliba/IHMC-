package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.Duration;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

import java.io.IOException;

public class DurationSubscriberExample
{
   public static void subscribeToTopic() throws IOException, InterruptedException
   {
      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "Ros2ListenerExample");
      node.createSubscription(Duration.getPubSubType().get(), subscriber -> {
         Duration durationMessage = new Duration();
         if (subscriber.takeNextData(durationMessage, null)) {
            System.out.println(durationMessage.getSec());
         }
      }, "duration_topic");

      Thread.currentThread().join(); // keep thread alive to receive more messages
   }

   public static void main(String[] args)
   {
      try
      {
         subscribeToTopic();
      }
      catch (Exception e)
      {
      }

   }

}