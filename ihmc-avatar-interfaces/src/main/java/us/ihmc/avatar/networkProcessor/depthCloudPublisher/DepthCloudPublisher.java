package us.ihmc.avatar.networkProcessor.depthCloudPublisher;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.DepthCloudMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher.StereoVisionWorldTransformCalculator;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class DepthCloudPublisher
{
   private static final boolean Debug = true;

   private static final int MAX_NUMBER_OF_POINTS = 200000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<PointCloudData> rosDepthCloud2ToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullRobotModel fullRobotModel;
   private DepthCloudWorldTransformCalculator depthCloudTransformer = null;
   private final RigidBodyTransform worldTransformer = new RigidBodyTransform();
   private final Pose3D sensorPose = new Pose3D();

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private RobotROSClockCalculator rosClockCalculator = null;

   private final IHMCROS2Publisher<DepthCloudMessage> depthcloudPublisher;
   private final IHMCRealtimeROS2Publisher<DepthCloudMessage> depthcloudRealtimePublisher;

   public DepthCloudPublisher(FullRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node, null, robotConfigurationDataTopicName);
   }

   public DepthCloudPublisher(String robotName, FullRobotModel fullRobotModel, Ros2Node ros2Node, RealtimeRos2Node realtimeRos2Node,
                              String robotConfigurationDataTopicName)
   {
      this.robotName = robotName;
      this.fullRobotModel = fullRobotModel;

      if (ros2Node != null)
      {
         ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         depthcloudPublisher = ROS2Tools.createPublisher(ros2Node, DepthCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
         depthcloudRealtimePublisher = null;
      }
      else
      {
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         depthcloudPublisher = null;
         depthcloudRealtimePublisher = ROS2Tools.createPublisher(realtimeRos2Node, DepthCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

      }
   }

   public void setROSClockCalculator(RobotROSClockCalculator rosClockCalculator)
   {
      this.rosClockCalculator = rosClockCalculator;
   }

   public void receiveStereoPointCloudFromROS(String stereoPointCloudROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(stereoPointCloudROSTopic, createROSPointCloud2Subscriber());
   }

   private RosPointCloudSubscriber createROSPointCloud2Subscriber()
   {
      return new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            rosDepthCloud2ToPublish.set(new PointCloudData(pointCloud, MAX_NUMBER_OF_POINTS));

            if (Debug)
               System.out.println("Receiving point cloud, n points: " + pointCloud.getHeight() * pointCloud.getWidth());
         }
      };
   }
   
   public void setCustomDepthCameraTransformer(DepthCloudWorldTransformCalculator transformer)
   {
      depthCloudTransformer = transformer;
   }

   public void start()
   {
      publisherTask = executorService.scheduleAtFixedRate(this::readAndPublishInternal, 0L, 1L, TimeUnit.MILLISECONDS);
   }

   public void shutdown()
   {
      publisherTask.cancel(false);
      executorService.shutdownNow();
   }

   private void readAndPublishInternal()
   {
      try
      {
         transformDataAndPublish();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         executorService.shutdown();
      }
   }

   private void transformDataAndPublish()
   {
      PointCloudData depthCloudData = rosDepthCloud2ToPublish.getAndSet(null);

      if (depthCloudData == null)
         return;

      long robotTimestamp;

      if (rosClockCalculator == null)
      {
         robotTimestamp = depthCloudData.getTimestamp();
         robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
      }
      else
      {
         long rosTimestamp = depthCloudData.getTimestamp();
         robotTimestamp = rosClockCalculator.computeRobotMonotonicTime(rosTimestamp);
         boolean waitForTimestamp = true;
         if (robotConfigurationDataBuffer.getNewestTimestamp() == -1)
            return;

         boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;

         if (!success)
            return;
      }

      if (depthCloudTransformer != null)
      {
         depthCloudTransformer.computeTransformToWorld(fullRobotModel, worldTransformer, sensorPose);
         depthCloudData.applyTransform(worldTransformer);
      }

      DepthCloudMessage message = depthCloudData.toDepthCloudMessage();
      message.getSensorPosition().set(sensorPose.getPosition());
      message.getSensorOrientation().set(sensorPose.getOrientation());

      if (Debug)
         System.out.println("Publishing stereo data, number of points: " + (message.getPointCloud().size() / 3));
      if (depthcloudPublisher != null)
         depthcloudPublisher.publish(message);
      else
         depthcloudRealtimePublisher.publish(message);
   }

   public static interface DepthCloudWorldTransformCalculator
   {
      public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform worldTransformer, Pose3DBasics sensorPoseToPack);
   }
}
