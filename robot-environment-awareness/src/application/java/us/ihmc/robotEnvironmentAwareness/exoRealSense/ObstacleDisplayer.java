package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.net.URISyntaxException;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

/*
 * main class that connect realsense D415 (RosNodeWithD415BridgeToRos2), runs point cloud through planar region finder (todo JOBY), evaluate obstacles (todo JOBY) and present them to hololense
 */
public class ObstacleDisplayer
{
   //variables
   private static final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());
   
   
   public static void main(String[] args) throws URISyntaxException
   {
      new RosNodeWithD415BridgeToRos2(ros2Node); //connection to realsense D415
      
      
      
   }
   
}