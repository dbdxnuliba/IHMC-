package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferInt;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ImageMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
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
import us.ihmc.robotics.controllers.AbstractPIDController;
import us.ihmc.ros2.Ros2Node;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final Ros2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;

   private Socket socketForObjectDetector;
   private static final String socketHostIPAddress = "127.0.0.1";
   private static final int socketPort = 65535;

   //   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   //   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;

   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);

   private final FusionSensorObjectDetectionManager objectDetectionManager;
   private final AtomicReference<List<ObjectType>> selectedObjecTypes;

   private LidarImageFusionProcessorCommunicationModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.ros2Node = ros2Node;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);

      ROS2Tools.createCallbackSubscription(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan", this::dispatchLidarScanMessage);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           StereoVisionPointCloudMessage.class,
                                           "/ihmc/stereo_vision_point_cloud",
                                           this::dispatchStereoVisionPointCloudMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, ImageMessage.class, "/ihmc/image", this::dispatchImageMessage);

      objectDetectionManager = new FusionSensorObjectDetectionManager(ros2Node);

      messager.registerTopicListener(LidarImageFusionAPI.RequestSocketConnection, (content) -> connectWithObjectDetectionModule());
      messager.registerTopicListener(LidarImageFusionAPI.RequestObjectDetection, (content) -> requestObjectDetection());
      selectedObjecTypes = messager.createInput(LidarImageFusionAPI.SelectedObjecTypes, new ArrayList<ObjectType>());
   }

   private void connectWithObjectDetectionModule()
   {
      System.out.println("connectWithObjectDetectionModule");

      try
      {
         socketForObjectDetector = new Socket(socketHostIPAddress, socketPort);
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void requestObjectDetection()
   {
      System.out.println("requestObjectDetection");
      //messager.submitMessage(LidarImageFusionAPI.RequestObjectDetection, false);

      BufferedImage imageToSend = latestBufferedImage.getAndSet(null);
      System.out.println("imageToSend " + imageToSend.getWidth() + " " + imageToSend.getHeight());

      for (ObjectType type : selectedObjecTypes.get())
      {
         System.out.println(type.toString());
      }
      // TODO : send it to server here.
      int numberOfObjectToDetect = selectedObjecTypes.get().size();
      if (numberOfObjectToDetect != 0)
      {
         List<RegionOfInterest> detectedROIs = new ArrayList<>();
         try
         {
            DataBuffer dataBuffer = imageToSend.getData().getDataBuffer();
            int[] imageInts = ((DataBufferInt) dataBuffer).getData();
            ByteBuffer byteBuffer = ByteBuffer.allocate(imageInts.length * 4);
            IntBuffer intBuffer = byteBuffer.asIntBuffer();
            intBuffer.put(imageInts);
            byte[] imageBytes = byteBuffer.array();
            System.out.println(imageBytes.length);

            OutputStream outputStream = socketForObjectDetector.getOutputStream();
            DataOutputStream dos = new DataOutputStream(outputStream);
            if(dos == null)
               System.out.println("hey null!");
            dos.write(imageBytes);
            
            BufferedReader br = new BufferedReader(new InputStreamReader(socketForObjectDetector.getInputStream()));
           /**
            * result = [xmin xmax ymin ymax] 
            */
            int[] result = Arrays.stream(br.readLine().split(",")).mapToInt(Integer::parseInt).toArray();
            LogTools.info("aaaaaaaaaaaaaaaaaaaaaa");
            System.out.println(Arrays.toString(result));
            RegionOfInterest roi = new RegionOfInterest();
            roi.setXOffset(result[0]);
            roi.setYOffset(result[2]);
            roi.setHeight(result[3] - result[2]);
            roi.setWidth(result[1] - result[0]);
            detectedROIs.add(roi);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         objectDetectionManager.computeAndPublish(ObjectType.Door, detectedROIs.get(0));
      }
      
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
      objectDetectionManager.updateLatestStereoVisionPointCloudMessage(message);
   }

   private void dispatchImageMessage(Subscriber<ImageMessage> subscriber)
   {
      ImageMessage message = subscriber.takeNextData();
      if (messager.isMessagerOpen())
         messager.submitMessage(LidarImageFusionAPI.ImageState, new ImageMessage(message));

      latestBufferedImage.set(FusionSensorImageViewer.convertImageMessageToBufferedImage(message));
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

   public static LidarImageFusionProcessorCommunicationModule createIntraprocessModule(SharedMemoryJavaFXMessager messager,
                                                                                       DomainFactory.PubSubImplementation implementation)
         throws IOException
   {
      KryoMessager kryoMessager = KryoMessager.createIntraprocess(REAModuleAPI.API,
                                                                  NetworkPorts.REA_MODULE_UI_PORT,
                                                                  REACommunicationProperties.getPrivateNetClassList());
      kryoMessager.setAllowSelfSubmit(true);
      kryoMessager.startMessager();

      Ros2Node ros2Node = ROS2Tools.createRos2Node(implementation, "ihmc_lidar_image_fusion_ui");
      return new LidarImageFusionProcessorCommunicationModule(ros2Node, kryoMessager, messager);
   }
}
