package us.ihmc.robotEnvironmentAwareness.fusion;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;

public class LidarImageFusionProcessorModule
{
   private final Ros2Node ros2Node;// = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2_ID.getNodeName());

   private LidarImageFusionProcessorModule(Ros2Node ros2Node, Messager reaMessager)
   {
      this.ros2Node = ros2Node;
   }


   public static LidarImageFusionProcessorModule createIntraprocessModule(SharedMemoryJavaFXMessager messager,
                                                                          DomainFactory.PubSubImplementation implementation)
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(implementation, "ihmc_lidar_image_fusion_ui");
      return new LidarImageFusionProcessorModule(ros2Node, messager);
   }
}
