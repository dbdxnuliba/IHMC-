package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class RealTimePlanarRegionSLAMUITabController
{
   private Messager messager;
   
   private PlanarRegionSLAMResultViewer slamResultViewer;
   private Group root = new Group();
   
   @FXML private ToggleButton enableIncomingPlanarRegions;
   @FXML private Button importingPlanarRegions;
   @FXML private Button clearPlanarRegions;
   @FXML private TextField planarRegionUpdaterStatus;
   
   @FXML private ToggleButton enableSLAM;
   @FXML private ToggleButton visualizeResult;
   @FXML private ToggleButton visualizeMap;
   @FXML private Button clearMap;
   
   public void initialize(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      slamResultViewer = new PlanarRegionSLAMResultViewer(ros2Node, messager);
   }
   
   public Node getRoot()
   {
      return root;
   }
}
