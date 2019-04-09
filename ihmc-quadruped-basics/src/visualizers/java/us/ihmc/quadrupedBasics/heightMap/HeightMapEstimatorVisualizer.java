package us.ihmc.quadrupedBasics.heightMap;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;


public class HeightMapEstimatorVisualizer
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;


   @FXML
   private HeightMapEstimatorController heightMapEstimatorController;


   public HeightMapEstimatorVisualizer(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      heightMapEstimatorController.attachMessager(messager, HeightMapEstimatorMessagerAPI.PointToAddTopic, HeightMapEstimatorMessagerAPI.PlanarRegionDataTopic);

//      heightMapEstimatorController.bindControls();


      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, HeightMapEstimatorMessagerAPI.PlanarRegionDataTopic,
                                                       HeightMapEstimatorMessagerAPI.ShowPlanarRegionsTopic);


      view3dFactory.addNodeToView(planarRegionViewer.getRoot());





      planarRegionViewer.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }



   public JavaFXMessager getMessager()
   {
      return messager;
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      planarRegionViewer.stop();
   }


   public static HeightMapEstimatorVisualizer createMessagerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return new HeightMapEstimatorVisualizer(primaryStage, messager);
   }
}
