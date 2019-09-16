package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;

public class RealTimePlanarRegionSLAMLauncher extends Application
{
   private SharedMemoryJavaFXMessager messager;

   private RealTimePlanarRegionSLAMUI ui;
   private RealTimePlanarRegionSLAMModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "real_time_planar_region_slam_ui");
      messager = new SharedMemoryJavaFXMessager(RealTimePlanarRegionSLAMAPI.API);
      messager.startMessager();
      module = RealTimePlanarRegionSLAMModule.createIntraprocessModule(ros2Node, messager);
      ui = new RealTimePlanarRegionSLAMUI(ros2Node, messager, primaryStage);
      
      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      messager.closeMessager();
      ui.stop();
      module.stop();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
