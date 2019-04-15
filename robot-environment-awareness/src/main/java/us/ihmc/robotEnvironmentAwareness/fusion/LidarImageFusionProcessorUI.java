package us.ihmc.robotEnvironmentAwareness.fusion;


import java.io.FileInputStream;

import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
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
      mainPane = loader.load();

      this.messager = messager;
      messager.startMessager();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      
      String imageLocation = "../../../../ihmc.jpg";
      FileInputStream fis = new FileInputStream(imageLocation);
      
      ImageView imagePane = new ImageView();
      Image sampleImage = new Image(fis);
      imagePane.setImage(sampleImage);
      
      GridPane centerPane = new GridPane();
      centerPane.getChildren().add(imagePane);
      
      mainPane.setRight(centerPane);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

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
