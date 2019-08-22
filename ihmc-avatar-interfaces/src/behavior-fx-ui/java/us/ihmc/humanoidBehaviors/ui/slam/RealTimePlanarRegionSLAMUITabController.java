package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class RealTimePlanarRegionSLAMUITabController
{
   private Messager messager;

   @FXML
   private ToggleButton enableIncomingPlanarRegions;
   @FXML
   private TextField planarRegionUpdaterStatus;
   @FXML
   private ToggleButton enableSLAM;
   @FXML
   private ToggleButton visualizeResult;
   @FXML
   private ToggleButton visualizeMap;

   public void initialize(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;

      messager.bindBidirectional(RealTimePlanarRegionSLAMAPI.EnablePlanarRegionIncoming, enableIncomingPlanarRegions.selectedProperty(), true);
      messager.bindBidirectional(RealTimePlanarRegionSLAMAPI.PlanarRegionStatus, planarRegionUpdaterStatus.textProperty(), true);

      messager.bindBidirectional(RealTimePlanarRegionSLAMAPI.EnableSLAM, enableSLAM.selectedProperty(), true);
      messager.bindBidirectional(RealTimePlanarRegionSLAMAPI.ShowSLAMResult, visualizeResult.selectedProperty(), true);
      messager.bindBidirectional(RealTimePlanarRegionSLAMAPI.ShowSLAMMap, visualizeMap.selectedProperty(), true);
   }

   public void clearPlanarRegions()
   {
      messager.submitMessage(RealTimePlanarRegionSLAMAPI.ClearPlanarRegion, true);
   }

   public void clearSLAMMap()
   {
      messager.submitMessage(RealTimePlanarRegionSLAMAPI.ClearSLAMMap, true);
   }

   public void importingPlanarRegions()
   {
      //TODO: refer SimulatedStereoVisionPointCloudPublisher
   }
}
