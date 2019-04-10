package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import controller_msgs.msg.dds.LidarScanMessage;
import sensor_msgs.Image;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class MultisenseImageReceiver extends AbstractRosTopicSubscriber<Image>
{
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "lidarScanPublisherNode");

   private final IHMCROS2Publisher<LidarScanMessage> lidarScanPublisher;
   
   public MultisenseImageReceiver() throws URISyntaxException, IOException
   {
      super(Image._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      rosMainNode.attachSubscriber("/multisense/left/image_rect_color", this);
      rosMainNode.execute();
      
      lidarScanPublisher = ROS2Tools.createPublisher(ros2Node, LidarScanMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
   }

   @Override
   public void onNewMessage(Image image)
   {
      System.out.println("received "+image.getWidth()+" "+image.getHeight());
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisenseImageReceiver();
   }
}