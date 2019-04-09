package us.ihmc.quadrupedBasics.heightMap;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

import java.util.concurrent.atomic.AtomicBoolean;

public class HeightMapEstimatorVisualizerApplication extends Application
{
   private static HeightMapEstimatorVisualizer ui;
   private static SharedMemoryJavaFXMessager messager;
   private static AtomicBoolean uiHasBeenConstructed = new AtomicBoolean(false);

   public HeightMapEstimatorVisualizerApplication()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(UIVisibilityGraphsTopics.API);
      messager.startMessager();

      ui = new HeightMapEstimatorVisualizer(primaryStage, messager);
      ui.show();

      uiHasBeenConstructed.set(true);
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
   }

   public HeightMapEstimatorVisualizer getUI()
   {
      return ui;
   }

   public SharedMemoryJavaFXMessager getMessager()
   {
      return messager;
   }

   public void startOnAThread()
   {
      Thread thread = new Thread(() -> launch());
      thread.start();

      LogTools.info("Waiting for UI to be constructed.");
      while (!uiHasBeenConstructed.get())
      {
         ThreadTools.sleep(100L);
      }

      LogTools.info("UI is constructed.");
   }
}
