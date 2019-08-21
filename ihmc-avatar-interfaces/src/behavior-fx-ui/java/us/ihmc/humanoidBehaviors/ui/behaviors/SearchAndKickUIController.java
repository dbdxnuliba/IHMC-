package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.*;
import javafx.scene.control.*;
import us.ihmc.humanoidBehaviors.*;
import us.ihmc.messager.Messager;

public class SearchAndKickUIController
{
   @FXML
   private CheckBox enableCheckBox;

   @FXML
   private Button startstepping;

   private Messager behaviorMessager;

   public void init(Messager behaviorMessager)
   {
      this.behaviorMessager = behaviorMessager;
   }

   @FXML
   public void enable()
   {
      behaviorMessager.submitMessage(SearchAndKickBehaviornewFramework.API.Enable, enableCheckBox.isSelected());
   }

   @FXML
   public void stepping()
   {
      behaviorMessager.submitMessage(SearchAndKickBehaviornewFramework.API.SearchAndKick, false);
   }
}
