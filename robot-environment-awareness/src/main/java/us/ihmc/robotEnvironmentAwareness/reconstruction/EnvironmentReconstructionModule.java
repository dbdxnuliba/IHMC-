package us.ihmc.robotEnvironmentAwareness.reconstruction;

import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.ros2.Ros2Node;

public class EnvironmentReconstructionModule
{
   private final Ros2Node ros2Node;
   private final SharedMemoryJavaFXMessager messager;
   
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   
   public EnvironmentReconstructionModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.ros2Node = ros2Node;
      this.messager = messager;
      
      new ROS2Callback<>(ros2Node, LidarScanMessage.class, this::dispatchLidarScanMessage);
      new ROS2Callback<>(ros2Node, StereoVisionPointCloudMessage.class, this::dispatchStereoVisionPointCloudMessage);
   }
   
   private void dispatchLidarScanMessage(LidarScanMessage message)
   {
      messager.submitMessage(EnvironmentReconstructionAPI.LidarScanState, new LidarScanMessage(message));
   }

   private void dispatchStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      messager.submitMessage(EnvironmentReconstructionAPI.StereoVisionPointCloudState, new StereoVisionPointCloudMessage(message));
   }
   
   private void mainUpdate()
   {
      //System.out.println("mainUpdate");
   }
   
   public void start() throws IOException
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, 1000, TimeUnit.MILLISECONDS);
//         executorService.scheduleAtFixedRate(lidarBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }
   
   public void stop() throws Exception
   {
      messager.closeMessager();
      ros2Node.destroy();

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }
   
   public static EnvironmentReconstructionModule createIntraprocessModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      return new EnvironmentReconstructionModule(ros2Node, messager);
   }
}
