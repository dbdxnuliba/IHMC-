package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;

public class StereoFilterAnchorPaneController extends REABasicUIController
{
   @FXML
   private Slider stereoNumberOfBufferSlider;
   @FXML
   private Slider stereoBufferSizeSlider;
   @FXML
   private Slider surfaceNormalUpperBound;
   @FXML
   private Slider surfaceNormalLowerBound;

   @Override
   public void bindControls()
   {
      
   }
}
