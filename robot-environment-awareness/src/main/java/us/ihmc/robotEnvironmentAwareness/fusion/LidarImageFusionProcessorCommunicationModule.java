package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ImageMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.FusionSensorObjectDetectionManager;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectDetectionSocketHelper;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleStateReporter;
import us.ihmc.ros2.Ros2Node;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final Ros2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;

   private static final int socketPort = 65535;
   private static final long milliSecondsForOneTick = 10;
   private static final double maximumTimeToWaitResult = 20.0;
   private Socket objectDetectionSocket;
   private boolean imageRequested;
   private final Map<ObjectType, RegionOfInterest> objectTypeToROIMap = new HashMap<>();

   private final FusionSensorObjectDetectionManager objectDetectionManager;

   private final AtomicReference<String> socketHostIPAddress;
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);
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

      objectDetectionManager = new FusionSensorObjectDetectionManager(ros2Node);

      messager.registerTopicListener(LidarImageFusionAPI.RequestSocketConnection, (content) -> connectWithObjectDetectionModule());
      messager.registerTopicListener(LidarImageFusionAPI.RequestObjectDetection, (content) -> requestObjectDetection());
      selectedObjecTypes = messager.createInput(LidarImageFusionAPI.SelectedObjecTypes, new ArrayList<ObjectType>());
      socketHostIPAddress = messager.createInput(LidarImageFusionAPI.ObjectDetectionModuleAddress);

      TimerTask socketTimerTask = new TimerTask()
      {
         private double waitingTime = 0;

         @Override
         public void run()
         {
            if (!imageRequested)
               return;

            waitingTime += (double) milliSecondsForOneTick / 1000;
            if (waitingTime > maximumTimeToWaitResult)
            {
               done();
               System.out.println("# detecting time out!");
            }

            try
            {
               InputStreamReader inputStreamReader = new InputStreamReader(objectDetectionSocket.getInputStream());
               BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

               if (!bufferedReader.ready())
                  return;

               int[] roiData = ObjectDetectionSocketHelper.convertStringToIntArray(bufferedReader.readLine());
               objectTypeToROIMap.clear();
               for (int i = 0; i < roiData.length; i += 5)
               {
                  if (roiData[i] == -1)
                     return;
                  ObjectType detectedObjectType = ObjectType.values()[roiData[i]];
                  int xmin = roiData[i + 1];
                  int xmax = roiData[i + 2];
                  int ymin = roiData[i + 3];
                  int ymax = roiData[i + 4];
                  RegionOfInterest roi = new RegionOfInterest();
                  roi.setXOffset(xmin);
                  roi.setYOffset(ymin);
                  roi.setHeight(ymax - ymin);
                  roi.setWidth(xmax - xmin);
                  objectTypeToROIMap.put(detectedObjectType, roi);
               }
               System.out.println("# detecting time tooks "+waitingTime+" seconds.");
               done();
               reportROIResults();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }

         private void done()
         {
            waitingTime = 0.0;
            imageRequested = false;
         }
      };
      Timer socketTimer = new Timer();
      socketTimer.schedule(socketTimerTask, 0, milliSecondsForOneTick);
   }

   private void reportROIResults()
   {
      int numberOfSelectedObjects = selectedObjecTypes.get().size();
      for (int i = 0; i < numberOfSelectedObjects; i++)
      {
         ObjectType objectType = selectedObjecTypes.get().get(i);
         RegionOfInterest regionOfInterest = objectTypeToROIMap.get(objectType);
         if (regionOfInterest != null)
            System.out.println(objectType + " " + regionOfInterest.getXOffset() + " " + regionOfInterest.getYOffset() + " " + regionOfInterest.getWidth() + " "
                  + regionOfInterest.getHeight());
      }
      // TODO: report to FusionSensorObjectDetectionManager.
   }

   private void connectWithObjectDetectionModule()
   {
      System.out.println("server address to connect is " + socketHostIPAddress.get());

      try
      {
         objectDetectionSocket = new Socket(socketHostIPAddress.get(), socketPort);
      }
      catch (UnknownHostException | ConnectException e)
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
      BufferedImage imageToSend = latestBufferedImage.getAndSet(null);

      imageRequested = true;
      sendImageToObjectDetectionModule(imageToSend);
   }

   private void sendImageToObjectDetectionModule(BufferedImage bufferedImage)
   {
      byte[] imgBytes = ObjectDetectionSocketHelper.convertImgToBytes(bufferedImage);
      byte[] imgDimBytes = ObjectDetectionSocketHelper.convertImgDimToBytes(imgBytes.length, bufferedImage.getWidth(), bufferedImage.getHeight());
      try
      {
         DataOutputStream dataOutputStream = new DataOutputStream(objectDetectionSocket.getOutputStream());
         dataOutputStream.write(imgDimBytes);
         dataOutputStream.write(imgBytes);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

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
      messager.closeMessager();
      ros2Node.destroy();
      if (objectDetectionSocket != null)
         if (!objectDetectionSocket.isClosed())
            objectDetectionSocket.close();
   }

   public static LidarImageFusionProcessorCommunicationModule createIntraprocessModule(SharedMemoryJavaFXMessager messager,
                                                                                       DomainFactory.PubSubImplementation implementation)
         throws IOException
   {
      KryoMessager kryoMessager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                                  REACommunicationProperties.getPrivateNetClassList());
      kryoMessager.setAllowSelfSubmit(true);
      kryoMessager.startMessager();

      Ros2Node ros2Node = ROS2Tools.createRos2Node(implementation, "ihmc_lidar_image_fusion_ui");
      return new LidarImageFusionProcessorCommunicationModule(ros2Node, kryoMessager, messager);
   }
}
