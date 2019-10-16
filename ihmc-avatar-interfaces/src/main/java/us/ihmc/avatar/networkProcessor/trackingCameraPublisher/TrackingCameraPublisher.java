package us.ihmc.avatar.networkProcessor.trackingCameraPublisher;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.TrackingCameraMessage;
import geometry_msgs.Pose;
import us.ihmc.avatar.networkProcessor.depthCloudPublisher.DepthCloudPublisher;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosNavMsgsOdometrySubscriber;

public class TrackingCameraPublisher
{
   private static final boolean Debug = false;

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<TrackingCameraData> rosDataToPublish = new AtomicReference<>(null);

   private final FullRobotModel fullRobotModel;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private RobotROSClockCalculator rosClockCalculator = null;

   private final IHMCROS2Publisher<TrackingCameraMessage> trackingCameraPublisher;
   private final IHMCRealtimeROS2Publisher<TrackingCameraMessage> trackingCameraRealtimePublisher;

   private DepthCloudPublisher depthCloudPublisher = null;
   private TrackingCameraCustomOriginWorldTransform customOriginWorldTransform = null;
   
   public TrackingCameraPublisher(FullRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node, null, robotConfigurationDataTopicName);
   }

   public TrackingCameraPublisher(String robotName, FullRobotModel fullRobotModel, Ros2Node ros2Node, RealtimeRos2Node realtimeRos2Node,
                                  String robotConfigurationDataTopicName)
   {
      this.fullRobotModel = fullRobotModel;

      if (ros2Node != null)
      {
         ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         trackingCameraPublisher = ROS2Tools.createPublisher(ros2Node, TrackingCameraMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
         trackingCameraRealtimePublisher = null;
      }
      else
      {
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         trackingCameraPublisher = null;
         trackingCameraRealtimePublisher = ROS2Tools.createPublisher(realtimeRos2Node, TrackingCameraMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

      }
   }

   public void setROSClockCalculator(RobotROSClockCalculator rosClockCalculator)
   {
      this.rosClockCalculator = rosClockCalculator;
   }

   public void receiveNavigationMessageFromROS(String stereoPointCloudROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(stereoPointCloudROSTopic, createNavigationMessageSubscriber());
   }

   public void setDepthCloudPublisher(DepthCloudPublisher depthCloudPublisher)
   {
      this.depthCloudPublisher = depthCloudPublisher;
   }
   
   public void setTrackingCameraCustomOriginWorldTransform(TrackingCameraCustomOriginWorldTransform customOriginWorldTransform)
   {
      this.customOriginWorldTransform = customOriginWorldTransform;
   }

   private RosNavMsgsOdometrySubscriber createNavigationMessageSubscriber()
   {
      return new RosNavMsgsOdometrySubscriber()
      {
         @Override
         public void onNewMessage(nav_msgs.Odometry message)
         {
            long timeStamp = message.getHeader().getStamp().totalNsecs();

            if (Debug)
               System.out.println("Odometry timeStamp " + timeStamp);

            Pose pose = message.getPose().getPose();
            TrackingCameraData trackingCameraData = new TrackingCameraData(timeStamp);
            trackingCameraData.setPosition(pose.getPosition());
            trackingCameraData.setOrientation(pose.getOrientation()); // TODO: see Odometry for twist.

            if (Debug)
               System.out.println("message.getPose().getPose() " + message.getPose().getPose().getPosition().getX());

            rosDataToPublish.set(trackingCameraData);
         }

         @Override
         protected void newPose(String frameID, TimeStampedTransform3D transform)
         {

         }
      };
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
      TrackingCameraData trackingCameraData = rosDataToPublish.getAndSet(null);

      if (trackingCameraData == null)
         return;

      long robotTimestamp;

      if (rosClockCalculator == null)
      {
         robotTimestamp = trackingCameraData.getTimestamp();
         robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
      }
      else
      {
         long rosTimestamp = trackingCameraData.getTimestamp();
         robotTimestamp = rosClockCalculator.computeRobotMonotonicTime(rosTimestamp);
         boolean waitForTimestamp = true;
         if (robotConfigurationDataBuffer.getNewestTimestamp() == -1)
            return;

         boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;

         if (!success)
            return;
      }

      if(customOriginWorldTransform != null)
         trackingCameraData.transform(customOriginWorldTransform.getOriginWorldTransform());
      
      TrackingCameraMessage message = trackingCameraData.toTrackingCameraMessage();
      Pose3D sensorPose = new Pose3D();
      sensorPose.setPosition(message.getSensorPosition());
      sensorPose.setOrientation(message.getSensorOrientation());

      if (depthCloudPublisher != null && depthCloudPublisher.useEstimatedSensorPose())
         depthCloudPublisher.updateEstimatedSensorPose(sensorPose);

      if (Debug)
      {
         System.out.println("TrackingCameraMessage.");
         System.out.println(message.timestamp_);
         System.out.println(message.quality_);
         System.out.println(message.sensor_position_);
         System.out.println(message.sensor_orientation_);
      }

      if (trackingCameraPublisher != null)
         trackingCameraPublisher.publish(message);
      else
         trackingCameraRealtimePublisher.publish(message);
   }

   public static interface TrackingCameraCustomOriginWorldTransform
   {
      public void initialize(FullRobotModel fullRobotModel, RigidBodyTransform worldTransformToPack);

      public RigidBodyTransform getOriginWorldTransform();
   }
}
