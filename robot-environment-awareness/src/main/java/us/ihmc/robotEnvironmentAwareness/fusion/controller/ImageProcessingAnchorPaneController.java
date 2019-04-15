package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionAPI;

public class ImageProcessingAnchorPaneController
{
   @FXML
   private Button btnTaskingSnapshot;

   @FXML
   private ToggleButton btnEnableStreaming;

   public void initialize(JavaFXMessager messager)
   {
      btnTaskingSnapshot.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            System.out.println("ImageProcessingAnchorPaneController true");
            messager.submitMessage(LidarImageFusionAPI.ImageSnapShot, true);
         }
      });

      messager.bindBidirectional(LidarImageFusionAPI.EnableStreaming, btnEnableStreaming.selectedProperty(), false);
   }
}
