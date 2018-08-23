package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.commons.PrintTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.ros2.Ros2Subscription;

import java.io.IOException;
import java.util.function.Supplier;

public class SubscriberExample
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      // Preallocate message for packing
      RobotConfigurationData robotConfigurationDataToListen = new RobotConfigurationData();

      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "MockAtlasController");
      Ros2Publisher<RobotConfigurationData> publisher = node.createPublisher(new RobotConfigurationDataPubSubType(), "/robot_configuration_data");
      Ros2Subscription<RobotConfigurationData> subs = node.createSubscription(new RobotConfigurationDataPubSubType(), subscriber -> {
         try
         {
            if (subscriber.takeNextData(robotConfigurationDataToListen, null))
            {
               // Access message data
               // long nanosec = robotConfigurationData.getHeader().getStamp().getNanosec();
               long nanosec = robotConfigurationDataToListen.getTimestamp();
            }
         }
         catch (Exception e)
         {
         }
      }, "/chatter");

      for (int i = 0; true; i++)
      {
         // Preallocate message data structure for packing
         RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

         // Pack message with data
         //         robotConfigurationData.getHeader().getStamp().setNanosec(i);
         robotConfigurationData.setTimestamp(i);
         System.out.println("Publishing timestamp: " + i);

         // Publish message, thread safe, copies data into another preallocated holder for sending
         publisher.publish(robotConfigurationData);


         Thread.sleep(1000);

         // listen to the published data
         //Supplier<TopicDataType> topicDataType = robotConfigurationDataToListen.getPubSubTypePacket();
         long receivedTimeStamp = robotConfigurationDataToListen.getTimestamp();
         PrintTools.info("Listening to time stamp: "+receivedTimeStamp);
      }
   }
}
