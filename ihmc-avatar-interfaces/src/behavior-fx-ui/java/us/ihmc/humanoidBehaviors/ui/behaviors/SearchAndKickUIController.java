package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.*;
import javafx.scene.control.*;
import us.ihmc.humanoidBehaviors.*;
import us.ihmc.humanoidBehaviors.SuppaKickBehavior.*;
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
      behaviorMessager.submitMessage(SearchAndKickBehavior.API.Enable, enableCheckBox.isSelected());
   }

   @FXML
   public void stepping()
   {
      behaviorMessager.submitMessage(SearchAndKickBehavior.API.SearchAndKick, false);
   }
}
