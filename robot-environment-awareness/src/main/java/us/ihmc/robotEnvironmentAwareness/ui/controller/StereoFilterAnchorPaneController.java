package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.StereoFilterParametersProperty;

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

   private final StereoFilterParametersProperty stereoFilterParametersProperty = new StereoFilterParametersProperty(this, "stereoFilterParametersProperty");

   @Override
   public void bindControls()
   {
      stereoFilterParametersProperty.bindBidirectionalNumberOfBuffer(stereoNumberOfBufferSlider.valueProperty());
      stereoFilterParametersProperty.bindBidirectionalSizeOfBuffer(stereoBufferSizeSlider.valueProperty());
      stereoFilterParametersProperty.bindBidirectionalSurfaceNormalUpperBound(surfaceNormalUpperBound.valueProperty());
      stereoFilterParametersProperty.bindBidirectionalSurfaceNormalLowerBound(surfaceNormalLowerBound.valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoFilterParameters, stereoFilterParametersProperty);
   }
}
