package us.ihmc.robotEnvironmentAwareness.reconstruction;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;

public class EnvironmentReconstructionStandaloneLauncher extends Application
{
   private SharedMemoryJavaFXMessager messager;

   private EnvironmentReconstructionUI ui;
   private EnvironmentReconstructionModule module;
   
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "environment_reconstruction_ui");
      messager = new SharedMemoryJavaFXMessager(EnvironmentReconstructionAPI.API);
      messager.startMessager();
      module = EnvironmentReconstructionModule.createIntraprocessModule(ros2Node, messager);
      ui = new EnvironmentReconstructionUI(ros2Node, messager, primaryStage);

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
