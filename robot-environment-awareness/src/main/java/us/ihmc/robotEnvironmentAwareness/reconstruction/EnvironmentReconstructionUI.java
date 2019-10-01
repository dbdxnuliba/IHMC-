package us.ihmc.robotEnvironmentAwareness.reconstruction;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.reconstruction.controller.SourceAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.reconstruction.viewer.EnvironmentReconstructionMeshViewer;
import us.ihmc.ros2.Ros2Node;

public class EnvironmentReconstructionUI
{
   private final SharedMemoryMessager messager;
   
   private final EnvironmentReconstructionMeshViewer meshViewer;
   
   @FXML
   private SourceAnchorPaneController sourceAnchorPaneController;
   
   public EnvironmentReconstructionUI(Ros2Node ros2Node, SharedMemoryMessager messager, Stage primaryStage) throws Exception
   {
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      BorderPane mainPane = loader.load();
      
      sourceAnchorPaneController.initialize(messager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();
      SubScene subScene = view3dFactory.getSubScene();
      Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();
      
      meshViewer = new EnvironmentReconstructionMeshViewer(ros2Node, messager);
      view3dFactory.addNodeToView(meshViewer.getRoot());
      
      mainPane.setCenter(subSceneWrappedInsidePane);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1400, 900);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         messager.closeMessager();

         //meshViewer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static EnvironmentReconstructionUI creatIntraprocessUI(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager, Stage primaryStage) throws Exception
   {
      return new EnvironmentReconstructionUI(ros2Node, messager, primaryStage);
   }
}
