package us.ihmc.humanoidBehaviors.ui.slam;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.ros2.Ros2Node;

public class RealTimePlanarRegionSLAMUI
{
   private final SharedMemoryJavaFXMessager messager;
   private final Stage primaryStage;
   
   private final PlanarRegionSLAMResultViewer slamResultViewer;

   @FXML
   private RealTimePlanarRegionSLAMUITabController realTimePlanarRegionSLAMUITabController;

   public RealTimePlanarRegionSLAMUI(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager, Stage primaryStage) throws IOException
   {
      this.messager = messager;
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      BorderPane mainPane = loader.load();

      realTimePlanarRegionSLAMUITabController.initialize(ros2Node, messager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();
      SubScene subScene = view3dFactory.getSubScene();
      Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

      slamResultViewer = new PlanarRegionSLAMResultViewer(ros2Node, messager);
      view3dFactory.addNodeToView(slamResultViewer.getRoot());

      mainPane.setCenter(subSceneWrappedInsidePane);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1400, 900);

      primaryStage.setScene(mainScene);
      primaryStage.show();
   }

   public void stop() throws Exception
   {
      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void show()
   {
      primaryStage.show();
   }
}
