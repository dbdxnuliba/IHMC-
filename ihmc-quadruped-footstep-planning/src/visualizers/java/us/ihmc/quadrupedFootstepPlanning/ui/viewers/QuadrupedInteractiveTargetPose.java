package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.Group;
import javafx.scene.paint.Material;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Translate;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedInteractiveTargetPose
{
   public static final double DEFAULT_SIZE = 0.05;

   private final JavaFXMessager messager;
   private final QuadrupedTargetNode node = new QuadrupedTargetNode();

   public QuadrupedInteractiveTargetPose(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   

   public static class QuadrupedTargetNode extends Group
   {
      private final Sphere center = new Sphere(DEFAULT_SIZE);
      private final QuadrantDependentList<Sphere> feet = new QuadrantDependentList<>();

      private final DoubleProperty stanceLengthProperty = new SimpleDoubleProperty(this, "stanceLength", 0.1);
      private final DoubleProperty stanceWidthProperty = new SimpleDoubleProperty(this, "stanceLength", 0.1);
      private final DoubleProperty feetSizeProperty = new SimpleDoubleProperty(this, "feetSize", 0.025);
      private final BooleanProperty showProperty = new SimpleBooleanProperty(this, "show", false);

      private final Affine nodeAffine = new Affine();

      public QuadrupedTargetNode()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            Sphere foot = new Sphere(0.5 * DEFAULT_SIZE);
            foot.radiusProperty().bind(feetSizeProperty);
            Translate translate = new Translate();
            translate.xProperty().bind(stanceLengthProperty.multiply(robotQuadrant.isQuadrantInFront() ? 0.5 : -0.5));
            translate.yProperty().bind(stanceWidthProperty.multiply(robotQuadrant.isQuadrantOnLeftSide() ? 0.5 : -0.5));
            foot.getTransforms().add(translate);
            feet.put(robotQuadrant, foot);
         }

         getTransforms().add(nodeAffine);

         showProperty.addListener(new ChangeListener<Boolean>()
         {
            @Override
            public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
            {
               if (oldValue.booleanValue() == newValue.booleanValue())
                  return;

               if (newValue.booleanValue())
               {
                  getChildren().add(center);
                  getChildren().addAll(feet.values());
               }
               else
               {
                  getChildren().clear();
               }
            }
         });
      }

      public void setMaterial(Material material)
      {
         center.setMaterial(material);
         feet.values().forEach(foot -> foot.setMaterial(material));
      }

      public void show(boolean show)
      {
         showProperty.set(show);
      }

      public boolean isShown()
      {
         return showProperty.get();
      }

      public BooleanProperty showProperty()
      {
         return showProperty;
      }

      public Affine getNodeAffine()
      {
         return nodeAffine;
      }

      public void setFeetSize(double feetSize)
      {
         feetSizeProperty.set(feetSize);
      }

      public double getFeetSize()
      {
         return feetSizeProperty.get();
      }

      public DoubleProperty feetSizeProperty()
      {
         return feetSizeProperty;
      }

      public void setStanceLength(double stanceLength)
      {
         stanceLengthProperty.set(stanceLength);
      }

      public double getStanceLength()
      {
         return stanceLengthProperty.get();
      }

      public DoubleProperty stanceLengthProperty()
      {
         return stanceLengthProperty;
      }

      public void setStanceWidth(double stanceWidth)
      {
         stanceWidthProperty.set(stanceWidth);
      }

      public double getStanceWidth()
      {
         return stanceWidthProperty.get();
      }

      public DoubleProperty stanceWidthProperty()
      {
         return stanceWidthProperty;
      }
   }
}
