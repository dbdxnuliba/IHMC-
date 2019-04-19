package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionAPI;

public class ObjectDetectionAnchorPaneController
{
   @FXML
   private Button btnDetectDoor;
   
   @FXML
   private ToggleButton enableObjectDetectionButton;
   
   public void initialize(SharedMemoryJavaFXMessager messager)
   {
      btnDetectDoor.setOnAction(new EventHandler<ActionEvent>() 
      {
         @Override
         public void handle(ActionEvent event)
         {
            messager.submitMessage(LidarImageFusionAPI.DetectDoor, true);
         }
      });
   }
}
