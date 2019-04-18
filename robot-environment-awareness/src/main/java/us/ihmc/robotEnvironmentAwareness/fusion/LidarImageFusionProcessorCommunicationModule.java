package us.ihmc.robotEnvironmentAwareness.fusion;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ImageMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.FusionSensorObjectDetectionManager;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleStateReporter;
import us.ihmc.ros2.Ros2Node;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final Ros2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;
   
   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   
   private final FusionSensorObjectDetectionManager objectDetectionModule;
   private final AtomicReference<List<ObjectType>> selectedObjecTypes;

   private LidarImageFusionProcessorCommunicationModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.ros2Node = ros2Node;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);

      ROS2Tools.createCallbackSubscription(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan", this::dispatchLidarScanMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, "/ihmc/stereo_vision_point_cloud",
                                           this::dispatchStereoVisionPointCloudMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, ImageMessage.class, "/ihmc/image", this::dispatchImageMessage);
      
      objectDetectionModule = new FusionSensorObjectDetectionManager(ros2Node);
      
      messager.registerTopicListener(LidarImageFusionAPI.RequestSocketConnection, (content) -> connectWithObjectDetectionModule());
      messager.registerTopicListener(LidarImageFusionAPI.RequestObjectDetection, (content) -> requestObjectDetection());
      selectedObjecTypes = messager.createInput(LidarImageFusionAPI.SelectedObjecTypes, new ArrayList<ObjectType>());
   }
   
   private void connectWithObjectDetectionModule()
   {
      System.out.println("connectWithObjectDetectionModule");
      // TODO : connect socket.
   }
   
   private void requestObjectDetection()
   {
      System.out.println("requestObjectDetection");
      for(ObjectType type : selectedObjecTypes.get())
      {
         System.out.println(type.toString());
      }
      // TODO : get recent image and send it to server here.
   }
   
   // TODO : waiting roi results from server.
   // TODO : the result should be a list of roi.
   // TODO : and calculate object parameters.

   private void dispatchLidarScanMessage(Subscriber<LidarScanMessage> subscriber)
   {
      LidarScanMessage message = subscriber.takeNextData();
      moduleStateReporter.registerLidarScanMessage(message);
   }

   private void dispatchStereoVisionPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      moduleStateReporter.registerStereoVisionPointCloudMessage(message);
   }

   private void dispatchImageMessage(Subscriber<ImageMessage> subscriber)
   {
      ImageMessage message = subscriber.takeNextData();
      if (messager.isMessagerOpen())
         messager.submitMessage(LidarImageFusionAPI.ImageState, new ImageMessage(message));
   }

   public void start() throws IOException
   {

   }

   public void stop() throws Exception
   {
      LogTools.info("LidarImageFusionProcessorCommunicationModule is going down.");

      messager.closeMessager();
      ros2Node.destroy();
   }

   public static LidarImageFusionProcessorCommunicationModule createIntraprocessModule(SharedMemoryJavaFXMessager messager, DomainFactory.PubSubImplementation implementation) throws IOException
   {
      KryoMessager kryoMessager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      kryoMessager.setAllowSelfSubmit(true);
      kryoMessager.startMessager();

      Ros2Node ros2Node = ROS2Tools.createRos2Node(implementation, "ihmc_lidar_image_fusion_ui");
      return new LidarImageFusionProcessorCommunicationModule(ros2Node, kryoMessager, messager);
   }
}
