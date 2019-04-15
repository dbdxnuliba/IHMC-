package us.ihmc.robotEnvironmentAwareness.fusion;

import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;

public class LidarImageFusionProcessorUI
{
   private final Messager messager;

   private final BorderPane mainPane;

   private final Stage primaryStage;

   private LidarImageFusionProcessorUI(SharedMemoryJavaFXMessager messager, Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      System.out.println(loader.getLocation());
      System.out.println(getClass().getSimpleName());
      mainPane = loader.load();

      // Client
      this.messager = messager;
      messager.startMessager();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      //view3dFactory.addNodeToView(reaMeshViewer.getRoot());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public static LidarImageFusionProcessorUI creatIntraprocessUI(SharedMemoryJavaFXMessager messager, Stage primaryStage) throws Exception
   {
      return new LidarImageFusionProcessorUI(messager, primaryStage);
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {

   }

}
