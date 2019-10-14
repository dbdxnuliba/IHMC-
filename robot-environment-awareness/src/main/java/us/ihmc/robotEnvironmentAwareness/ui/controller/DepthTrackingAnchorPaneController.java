package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class DepthTrackingAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableDepth;

   @FXML
   private ToggleButton enableTracking;

   public DepthTrackingAnchorPaneController()
   {
   }

   @Override
   public void bindControls()
   {
      uiMessager.bindBidirectionalInternal(REAModuleAPI.DepthCloudEnable, enableDepth.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.TrackingCameraEnable, enableTracking.selectedProperty(), true);
   }

   @FXML
   public void clearDepth()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.DepthCloudClear, true);
   }

   @FXML
   public void clearTracking()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.TrackingCameraClear, true);
   }
}
