package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ImageMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.image.ImageView;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;

public class FusionSensorImageViewer implements Runnable
{
   private static final int HIGH_PACE_UPDATE_PERIOD = 100;
   
   private JavaFXMessager messager;
   private final AnimationTimer imageAnimation;
   
   protected final AtomicReference<ImageMessage> newImageMessageToView;
   private final AtomicReference<Boolean> enableStreaming;
   private final AtomicReference<Boolean> snapshot;
   
   protected final AtomicReference<ImageView> imgaveToView = new AtomicReference<>(null);

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CANCEL_AND_REPORT);
   
   
   public FusionSensorImageViewer(SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;

      enableStreaming = messager.createInput(LidarImageFusionAPI.EnableStreaming, false);
      snapshot = messager.createInput(LidarImageFusionAPI.ImageSnapShot, false);
      
      newImageMessageToView = messager.createInput(LidarImageFusionAPI.ImageState);
      
      imageAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if(snapshot.getAndSet(false))
               System.out.println("take snap shot!!");
//            lidarScanViewer.render();
//            stereoVisionPointCloudViewer.render();
         }
      };
      
      messager.registerMessagerStateListener(isMessagerOpen -> {
         if (isMessagerOpen)
            start();
         else
            sleep();
      });
      
   }
   
   private void unpackImage(ImageMessage imageMessage)
   {
      System.out.println("unpacking ");
      if (imageMessage == null)
         return;
      
      System.out.println("getWidth "+imageMessage.getWidth());
      ImageView newImageToView = new ImageView();
   }
   
   @Override
   public void run()
   {
      if(!enableStreaming.get())
         return;
      
      System.out.println("enableStreaming ");
      
      if (newImageMessageToView.get() == null)
         return;

      unpackImage(newImageMessageToView.getAndSet(null));
   }

   public void start()
   {
      imageAnimation.start();
      
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(this, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
   }
   
   public void sleep()
   {
      imageAnimation.stop();
   }
}
