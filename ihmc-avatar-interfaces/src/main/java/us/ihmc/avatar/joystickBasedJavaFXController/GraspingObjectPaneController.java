package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.scene.control.SingleSelectionModel;
import javafx.scene.control.Slider;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class GraspingObjectPaneController
{
   @FXML
   private Slider sliderSphereRadius;

   @FXML
   private Slider sliderCylinderRadius;

   @FXML
   private Slider sliderCylinderHeight;

   @FXML
   private Slider sliderTorusRadius;

   @FXML
   private Slider sliderTorusTube;

   @FXML
   private Slider sliderBoxLength;

   @FXML
   private Slider sliderBoxWidth;

   @FXML
   private Slider sliderBoxHeight;

   @FXML
   private TabPane tapPaneShape;

   private final AnimationTimer tabSelector;

   private AtomicReference<Integer> selectedTab;

   public GraspingObjectPaneController()
   {
      tabSelector = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            SingleSelectionModel<Tab> selectionModel = tapPaneShape.getSelectionModel();
            selectionModel.select(selectedTab.get());
         }
      };
   }

   public void initialize(JavaFXMessager messager)
   {
      messager.bindBidirectional(GraspingJavaFXTopics.SphereRadius, sliderSphereRadius.valueProperty(), createConverter(), true);

      messager.bindBidirectional(GraspingJavaFXTopics.CylinderRadius, sliderCylinderRadius.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.CylinderHeight, sliderCylinderHeight.valueProperty(), createConverter(), true);

      messager.bindBidirectional(GraspingJavaFXTopics.TorusRadius, sliderTorusRadius.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.TorusTubeRadius, sliderTorusTube.valueProperty(), createConverter(), true);

      messager.bindBidirectional(GraspingJavaFXTopics.BoxLength, sliderBoxLength.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.BoxWidth, sliderBoxWidth.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.BoxHeight, sliderBoxHeight.valueProperty(), createConverter(), true);

      selectedTab = messager.createInput(GraspingJavaFXTopics.SelectedShape, 0);

      tabSelector.start();
   }

   private PropertyToMessageTypeConverter<Double, Number> createConverter()
   {
      return new PropertyToMessageTypeConverter<Double, Number>()
      {
         @Override
         public Double convert(Number propertyValue)
         {
            return propertyValue.doubleValue();
         }

         @Override
         public Number interpret(Double messageContent)
         {
            return messageContent;
         }
      };
   }
}