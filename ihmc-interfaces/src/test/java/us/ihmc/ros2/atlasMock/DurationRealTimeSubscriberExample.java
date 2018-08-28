package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.Duration;
import controller_msgs.msg.dds.LidarScanMessage;
import org.apache.commons.lang3.SystemUtils;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.subscribers.*;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.*;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;



import java.io.IOException;


public class DurationRealTimeSubscriberExample
{
   public static int subscribeToTopic() throws IOException
   {
      PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ? // realtime threads only work on linux
            new PeriodicRealtimeThreadSchedulerFactory(20) :           // see https://github.com/ihmcrobotics/ihmc-realtime
            new PeriodicNonRealtimeThreadSchedulerFactory();                   // to setup realtime threads
      RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "NonRealtimeRos2PublishSubscribeExample", "");
      RealtimeRos2Publisher<Duration> publisher = node.createPublisher(Duration.getPubSubType().get(), "duration_topic");

      RealtimeRos2Subscription<Duration> subscription = node.createQueuedSubscription(Duration.getPubSubType().get(), "duration_topic");

      System.out.println(111111);

      Duration message = new Duration();
      //for (int i = 0; i < 10; i++)
      //{
      //   message.setSec(i);
      //   System.out.println(message.getSec()); // first message
      //   publisher.publish(message);  // publish
      //}

      System.out.println(111111);

      Duration incomingMessage = new Duration();
      while (!subscription.poll(incomingMessage))
      {
         ; // just waiting for the first message
      }
      System.out.println(incomingMessage); // first message
      int i = 1;
      //while (true)
      //{
      if (subscription.poll(incomingMessage))  // poll for new messages
      {
         //System.out.println(incomingMessage);
         //i++;
      }
      else
      {
         // no available messages
      }
      //}
      System.out.println(incomingMessage); // first message
      int val = incomingMessage.getSec();
      System.out.println(val); // first message
      //node.spin(); // start the realtime node thread

      return val;

   }

   public static void main(String[] args)
   {
      try
      {
         int mex = subscribeToTopic();
         System.out.println(mex);
      }
      catch (Exception e)
      {
      }
   }
}