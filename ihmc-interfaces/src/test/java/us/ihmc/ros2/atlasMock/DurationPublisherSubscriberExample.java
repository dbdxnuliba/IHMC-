package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.Duration;
import controller_msgs.msg.dds.DurationPubSubType;
import us.ihmc.commons.PrintTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.ros2.Ros2Subscription;

import java.io.IOException;

public class DurationPublisherSubscriberExample
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      // Preallocate message for packing
      Duration durationToListen = new Duration();

      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "MockAtlasController");
      Ros2Publisher<Duration> publisher = node.createPublisher(new DurationPubSubType(), "duration_topic");
      Ros2Publisher<Duration> clock_pub = node.createPublisher(new DurationPubSubType(), "clock");
      Ros2Publisher<Duration> parameter_events_pub = node.createPublisher(new DurationPubSubType(), "parameter_events");

      Ros2Subscription<Duration> subs = node.createSubscription(new DurationPubSubType(), subscriber -> {
         try
         {
            if (subscriber.takeNextData(durationToListen, null))
            {
               // Access message data
               // long nanosec = robotConfigurationData.getHeader().getStamp().getNanosec();
               long nanosec = durationToListen.getSec();
            }
         }
         catch (Exception e)
         {
         }
      }, "duration_topic");

      //Duration durationToPublish = new Duration();
      //Thread.sleep(1000);
      //PrintTools.info("Listening to time, sec: "+durationToListen.getSec()+" nanosec: "+ durationToListen.getNanosec());

      for (int i = 0; true; i++)
      {
         // Preallocate message data structure for packing
         Duration durationToPublish = new Duration();

         // Pack message with data
         //         robotConfigurationData.getHeader().getStamp().setNanosec(i);
         //durationToPublish.setSec(i);
         //System.out.println("Publishing timestamp: " + i);

         // Publish message, thread safe, copies data into another preallocated holder for sending
         //publisher.publish(durationToPublish);
         //clock_pub.publish(durationToPublish);
         //parameter_events_pub.publish(durationToPublish);


         Thread.sleep(1000);

         // listen to the published data
         //Supplier<TopicDataType> topicDataType = durationToListen.getPubSubTypePacket();
         double receivedTimeStamp = durationToListen.getSec();
         PrintTools.info("Listening to time, sec: "+durationToListen.getSec()+" nanosec: "+ durationToListen.getNanosec());
      }
   }
}
