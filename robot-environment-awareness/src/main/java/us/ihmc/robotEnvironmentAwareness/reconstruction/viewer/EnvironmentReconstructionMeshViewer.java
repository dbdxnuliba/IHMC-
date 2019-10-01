package us.ihmc.robotEnvironmentAwareness.reconstruction.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.reconstruction.EnvironmentReconstructionAPI;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.LidarScanViewer;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.ros2.Ros2Node;

public class EnvironmentReconstructionMeshViewer
{
   private final LidarScanViewer lidarScanViewer;
   private final StereoVisionPointCloudViewer stereoVisionPointCloudViewer;

   private final Group root = new Group();

   private final AnimationTimer renderMeshAnimation;
   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   public EnvironmentReconstructionMeshViewer(Ros2Node ros2Node, SharedMemoryMessager messager) throws Exception
   {
      lidarScanViewer = new LidarScanViewer(EnvironmentReconstructionAPI.LidarScanState, messager)
      {
         @Override
         protected Topic<Boolean> createEnableInput()
         {
            return EnvironmentReconstructionAPI.LidarScanEnable;
         }
         
         @Override
         protected Topic<Boolean> createClearInput()
         {
            return EnvironmentReconstructionAPI.LidarScanClear;
         }
      };
      stereoVisionPointCloudViewer = new StereoVisionPointCloudViewer(EnvironmentReconstructionAPI.StereoVisionPointCloudState, messager)
      {
         @Override
         protected Topic<Boolean> createEnableInput()
         {
            return EnvironmentReconstructionAPI.StereoPointCloudEnable;
         }
         
         @Override
         protected Topic<Boolean> createClearInput()
         {
            return EnvironmentReconstructionAPI.StereoPointCloudClear;
         }
      };
      stereoVisionPointCloudViewer.enablePreserve(true);

      Node lidarScanRootNode = lidarScanViewer.getRoot();
      lidarScanRootNode.setMouseTransparent(true);
      Node stereoVisionPointCloudRootNode = stereoVisionPointCloudViewer.getRoot();
      stereoVisionPointCloudRootNode.setMouseTransparent(true);

      root.getChildren().addAll(lidarScanRootNode, stereoVisionPointCloudRootNode);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            lidarScanViewer.render();
            stereoVisionPointCloudViewer.render();

         }
      };
      start();
      
      messager.submitMessage(EnvironmentReconstructionAPI.StereoPointCloudEnable, true);
      messager.submitMessage(EnvironmentReconstructionAPI.LidarScanEnable, true);
   }

   public void start()
   {
      renderMeshAnimation.start();

      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(lidarScanViewer, 0, 100, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(stereoVisionPointCloudViewer, 0, 10, TimeUnit.MILLISECONDS));
   }

   public void sleep()
   {
      renderMeshAnimation.stop();
      meshBuilderScheduledFutures.clear();
   }

   public void stop()
   {
      sleep();
   }

   public Node getRoot()
   {
      return root;
   }
}
