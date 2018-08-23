package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import controller_msgs.msg.dds.RobotStateCartesianTrajectory;
import controller_msgs.msg.dds.RobotStateCartesianTrajectoryPubSubType;
import javafx.geometry.Point3D;
import us.ihmc.commons.PrintTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.ros2.Ros2Subscription;

import java.io.IOException;
import java.util.function.Supplier;

public class TowrSubscriberExample
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      // Preallocate message for packing
      RobotStateCartesianTrajectory robotStateCartesianTrajectoryToListen = new RobotStateCartesianTrajectory();


      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "MockAtlasController");
      //Ros2Publisher<RobotConfigurationData> publisher = node.createPublisher(new RobotConfigurationDataPubSubType(), "/robot_configuration_data");
      Ros2Subscription<RobotStateCartesianTrajectory> subs = node.createSubscription(new RobotStateCartesianTrajectoryPubSubType(), subscriber -> {
         try
         {
            if (subscriber.takeNextData(robotStateCartesianTrajectoryToListen, null))
            {
               PrintTools.info("heard topic!!!!");
               // Access message data
               // long nanosec = robotConfigurationData.getHeader().getStamp().getNanosec();
               us.ihmc.euclid.tuple3D.Point3D initial_base_pos = robotStateCartesianTrajectoryToListen.getPoints().get(0).base_.getPose().getPosition();
               PrintTools.info("initial pos:"+initial_base_pos);
            }
         }
         catch (Exception e)
         {
         }
      }, "/towr_ros2");

      for (int i = 0; true; i++)
      {
         // Preallocate message data structure for packing
         RobotConfigurationData robotConfigurationData = new RobotConfigurationData();
         // Pack message with data
         //         robotConfigurationData.getHeader().getStamp().setNanosec(i);
         robotConfigurationData.setTimestamp(i);
         //System.out.println("Publishing timestamp: " + i);

         // Publish message, thread safe, copies data into another preallocated holder for sending
         //publisher.publish(robotConfigurationData);


         Thread.sleep(1000);

         // listen to the published data
         //Supplier<TopicDataType> topicDataType = robotStateCartesianTrajectory.getPubSubTypePacket();
         //long receivedTimeStamp = robotConfigurationDataToListen.getTimestamp();
         //us.ihmc.euclid.tuple3D.Point3D initial_base_pos = robotStateCartesianTrajectoryToListen.getPoints().get(0).base_.getPose().getPosition();
         PrintTools.info("waiting for towr_ros2 topic"+robotStateCartesianTrajectoryToListen.getHeader().getFrameId());
      }
   }
}
