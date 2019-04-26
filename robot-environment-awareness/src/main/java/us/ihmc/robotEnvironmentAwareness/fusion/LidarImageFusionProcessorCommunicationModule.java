package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferInt;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.Socket;
import java.net.SocketException;
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
import us.ihmc.ros2.Ros2Node;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final Ros2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;

   private Socket socketForObjectDetector;
   private List<RegionOfInterest> detectedROIs;
   private final AtomicReference<String> socketHostIPAddress;
   private static final int socketPort = 65535;

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
      socketHostIPAddress = messager.createInput(LidarImageFusionAPI.ObjectDetectionModuleAddress);
   }

   private void connectWithObjectDetectionModule()
   {
      System.out.println("server address to connect is "+ socketHostIPAddress.get());

      try
      {
         socketForObjectDetector = new Socket(socketHostIPAddress.get(), socketPort);
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
      System.out.println("imageToSend " + imageToSend.getWidth() + " " + imageToSend.getHeight());

      for (ObjectType type : selectedObjecTypes.get())
      {
         System.out.println(type.toString());
      }
      // TODO : send it to server here.
      detectedROIs = sendImgAndGetRois(imageToSend);
      objectDetectionManager.computeAndPublish(ObjectType.Door, detectedROIs.get(0));
   }

/////////////////////////////////Abstraction////////////////////////////////////////////////////

   public List<RegionOfInterest> sendImgAndGetRois(BufferedImage bufferedImage)
   {
      byte[] imgBytes;
      byte[] imgDimBytes;
      DataOutputStream dataOutputStream;
      BufferedReader bufferedReader;
      RegionOfInterest roi = new RegionOfInterest();
      List<RegionOfInterest> roisToAppend = new ArrayList<>();
      int[] roiData;
      try
      {
         imgBytes = convertImgToBytes(bufferedImage);
         imgDimBytes = convertImgDimToBytes(imgBytes.length, bufferedImage.getWidth(), bufferedImage.getHeight());
         dataOutputStream = new DataOutputStream(socketForObjectDetector.getOutputStream());
         dataOutputStream.write(imgDimBytes);
         dataOutputStream.write(imgBytes);

         bufferedReader = new BufferedReader(new InputStreamReader(socketForObjectDetector.getInputStream()));
         roiData = convertStringToIntArray(bufferedReader.readLine());
         System.out.println(Arrays.toString(roiData));
         for (int i = 0; i < roiData.length; i+=5)
         {
            System.out.println(objNumToName(roiData[i])+" has been detected");
            // id, x min x max, y min, y max
            //roi.setName(roiData[i]);    add a name attribute
            int xmin = i+1;
            int xmax = i+2;
            int ymin = i+3;
            int ymax = i+4;
            roi.setXOffset(roiData[xmin]);
            roi.setYOffset(roiData[ymin]);
            roi.setHeight(roiData[ymax] - roiData[ymin]);
            roi.setWidth(roiData[xmax] - roiData[xmin]);
            roisToAppend.add(roi);
         }
         // TODO:
         // override roi pixels on bufferedImage.
//         messager.submitMessage(LidarImageFusionAPI.ImageResultState, bufferedImage);
         
      }
      catch (SocketException e)
      {
         System.out.println("hallo");
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      return roisToAppend;
   }

   public String objNumToName(int objNum)
   {
      String objName = "";
      switch(objNum)
      {
         case 0: objName = "no object";
            break;
         case 1: objName = "door";
            break;
         case 2: objName = "doorHandle";
            break;
      }
      return objName;
   }

   public int[] convertStringToIntArray(String msgFromPython)
   {
      int[] intArray = Arrays.stream(msgFromPython.split(",")).mapToInt(Integer::parseInt).toArray();
      return intArray;
   }

   public byte[] convertImgToBytes(BufferedImage bufferedImage)
   {
      int width = bufferedImage.getWidth();
      int height = bufferedImage.getHeight();

      DataBuffer dataBuffer = bufferedImage.getData().getDataBuffer();
      int[] imageInts = ((DataBufferInt) dataBuffer).getData();
      ByteBuffer byteBuffer = ByteBuffer.allocate(imageInts.length * 4);
      IntBuffer intBuffer = byteBuffer.asIntBuffer();
      intBuffer.put(imageInts);
      byte[] imageBytes = byteBuffer.array();
      return imageBytes;
   }

   public byte[] convertImgDimToBytes(int size, int width, int height)
   {
      int[] imgDim = {size, width, height};
      ByteBuffer byteBuffer = ByteBuffer.allocate(imgDim.length * 4);
      IntBuffer intBuffer = byteBuffer.asIntBuffer();
      intBuffer.put(imgDim);
      byte[] imageSizeBytes = byteBuffer.array();
      return imageSizeBytes;
   }
   
/////////////////////////////////Abstraction////////////////////////////////////////////////////
   
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
      socketForObjectDetector.close();
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
