package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.NonInvertibleTransformException;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedInteractiveTargetPose
{
   public static final double DEFAULT_SIZE = 0.05;
   public static final double DEFAULT_ARROW_LENGTH = 5.0 * DEFAULT_SIZE;

   public static final PhongMaterial defaultUnselectedMaterial = new PhongMaterial(Color.RED);
   public static final PhongMaterial defaultSelectedMaterial = new PhongMaterial(Color.ORANGE.deriveColor(0.0, 1.0, 1.0, 0.7));

   private final QuadrupedTargetNode node = new QuadrupedTargetNode();

   public QuadrupedInteractiveTargetPose()
   {
   }

   public QuadrupedTargetNode getTargetNode()
   {
      return node;
   }

   public <O extends Orientation3DReadOnly> void configureWithMessager(JavaFXMessager messager, Topic<Boolean> showTopic, Topic<Boolean> selectedPositionTopic,
                                                                       Topic<Boolean> selectedOrientationTopic, Topic<Point3D> positionTopic,
                                                                       Topic<O> orientationTopic, Topic<QuadrupedXGaitSettingsReadOnly> xGaitSettingsTopic,
                                                                       Topic<PlanarRegionsList> planarRegionsTopic)
   {
      messager.bindPropertyToTopic(showTopic, node.showProperty());
      messager.bindPropertyToTopic(selectedPositionTopic, node.positionSelectedProperty());
      messager.bindPropertyToTopic(selectedOrientationTopic, node.orientationSelectedProperty());
      messager.registerJavaFXSyncedTopicListener(positionTopic, m ->
      {
         if (m != null)
            node.positionProperty().set(new Point3D(m));
      });
      messager.registerJavaFXSyncedTopicListener(orientationTopic, m ->
      {
         if (m != null)
            node.orientationProperty().set(new Quaternion(m));
      });
      messager.registerJavaFXSyncedTopicListener(xGaitSettingsTopic, new TopicListener<QuadrupedXGaitSettingsReadOnly>()
      {
         @Override
         public void receivedMessageForTopic(QuadrupedXGaitSettingsReadOnly messageContent)
         {
            if (messageContent == null)
               return;
            node.setStanceLength(messageContent.getStanceLength());
            node.setStanceWidth(messageContent.getStanceWidth());
         }
      });

      AtomicReference<PlanarRegionsList> planarRegionsInput = messager.createInput(planarRegionsTopic);
      node.setFootPositionAdjustment(in ->
      {
         PlanarRegionsList planarRegionsList = planarRegionsInput.get();
         if (planarRegionsList == null)
            return null;

         return PlanarRegionTools.projectPointToPlanesVertically(new Point3D(in.getX(), in.getY(), 100.0), planarRegionsList);
      });
   }

   public static class QuadrupedTargetNode extends Group
   {
      private final Sphere center = new Sphere(DEFAULT_SIZE);
      private final QuadrantDependentList<Sphere> feet = new QuadrantDependentList<>();
      private final QuadrantDependentList<Translate> feetAdjustmentTranslates = new QuadrantDependentList<>(new Translate(),
                                                                                                            new Translate(),
                                                                                                            new Translate(),
                                                                                                            new Translate());
      private final MeshView arrow = createArrowGraphic(DEFAULT_ARROW_LENGTH, 0.5 * DEFAULT_SIZE);

      private final DoubleProperty stanceLengthProperty = new SimpleDoubleProperty(this, "stanceLength", 0.1);
      private final DoubleProperty stanceWidthProperty = new SimpleDoubleProperty(this, "stanceLength", 0.1);
      private final DoubleProperty feetSizeProperty = new SimpleDoubleProperty(this, "feetSize", 0.025);
      private final BooleanProperty showProperty = new SimpleBooleanProperty(this, "show", false);

      private Material unselectedMaterial = defaultUnselectedMaterial;
      private Material selectedMaterial = defaultSelectedMaterial;
      private final BooleanProperty positionSelectedProperty = new SimpleBooleanProperty(this, "selected");
      private final BooleanProperty orientationSelectedProperty = new SimpleBooleanProperty(this, "selected");

      private Function<Point3DReadOnly, Point3DReadOnly> footPositionAdjustment = Function.identity();

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
            foot.getTransforms().add(feetAdjustmentTranslates.get(robotQuadrant));
            feet.put(robotQuadrant, foot);
            foot.mouseTransparentProperty().bind(positionSelectedProperty.or(orientationSelectedProperty));
         }
         center.mouseTransparentProperty().bind(positionSelectedProperty.or(orientationSelectedProperty));
         arrow.mouseTransparentProperty().bind(positionSelectedProperty.or(orientationSelectedProperty));

         getTransforms().add(nodeAffine);
         nodeAffine.setOnTransformChanged(e -> adjustFeetPosition());

         showProperty.addListener(new ChangeListener<Boolean>()
         {
            @Override
            public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
            {
               if (oldValue.booleanValue() == newValue.booleanValue())
                  return;

               if (newValue.booleanValue())
               {
                  getChildren().add(arrow);
                  getChildren().add(center);
                  getChildren().addAll(feet.values());
               }
               else
               {
                  getChildren().clear();
               }
            }
         });

         positionSelectedProperty.addListener((observable, oldValue, newValue) ->
         {
            if (newValue.booleanValue() != oldValue.booleanValue())
               setPositionMaterial(newValue.booleanValue() ? selectedMaterial : unselectedMaterial);
         });
         orientationSelectedProperty.addListener((observable, oldValue, newValue) ->
         {
            if (newValue.booleanValue() != oldValue.booleanValue())
               setOrientationMaterial(newValue.booleanValue() ? selectedMaterial : unselectedMaterial);
         });
         showProperty.set(true);
      }

      private MeshView createArrowGraphic(double length, double radius)
      {
         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

         double coneHeight = 0.10 * length;
         double coneRadius = 1.5 * radius;

         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0));
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0));

         MeshView arrow = new MeshView(meshBuilder.generateMesh());
         return arrow;
      }

      private void adjustFeetPosition()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            Sphere foot = feet.get(robotQuadrant);

            Translate translate = feetAdjustmentTranslates.get(robotQuadrant);
            javafx.geometry.Point3D localToParent = foot.localToScene(0.0, 0.0, 0.0);
            Point3D footPosition = new Point3D(localToParent.getX(), localToParent.getY(), localToParent.getZ());
            Point3DReadOnly adjustedPosition = footPositionAdjustment.apply(footPosition);

            if (adjustedPosition == null)
            {
               translate.setX(0.0);
               translate.setY(0.0);
               translate.setZ(0.0);
            }
            else
            {
               Vector3D adjustmentVector = new Vector3D();
               adjustmentVector.sub(adjustedPosition, footPosition);

               if (robotQuadrant == RobotQuadrant.FRONT_LEFT)
                  LogTools.info(adjustmentVector.toString());

               try
               {
                  javafx.geometry.Point3D localOffset = foot.getLocalToSceneTransform()
                                                            .inverseDeltaTransform(adjustmentVector.getX(), adjustmentVector.getY(), adjustmentVector.getZ());
                  translate.setX(translate.getX() + localOffset.getX());
                  translate.setY(translate.getY() + localOffset.getY());
                  translate.setZ(translate.getZ() + localOffset.getZ());
               }
               catch (NonInvertibleTransformException e)
               {
                  e.printStackTrace();
                  translate.setX(0.0);
                  translate.setY(0.0);
                  translate.setZ(0.0);
               }
            }
         }
      }

      public void setFootPositionAdjustment(Function<Point3DReadOnly, Point3DReadOnly> function)
      {
         footPositionAdjustment = function;
      }

      public void setPositionMaterial(Material material)
      {
         center.setMaterial(material);
         feet.values().forEach(foot -> foot.setMaterial(material));
      }

      public void setOrientationMaterial(Material material)
      {
         arrow.setMaterial(material);
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

      public void setMaterials(Material unselectedMaterial, Material selectedMaterial)
      {
         this.unselectedMaterial = unselectedMaterial;
         this.selectedMaterial = selectedMaterial;
      }

      public boolean isPositionSelected()
      {
         return positionSelectedProperty.get();
      }

      public void setPositionSelected(boolean selected)
      {
         positionSelectedProperty.set(selected);
      }

      public BooleanProperty positionSelectedProperty()
      {
         return positionSelectedProperty;
      }

      public boolean isOrientationSelected()
      {
         return orientationSelectedProperty.get();
      }

      public void setOrientationSelected(boolean selected)
      {
         orientationSelectedProperty.set(selected);
      }

      public BooleanProperty orientationSelectedProperty()
      {
         return orientationSelectedProperty;
      }

      public Affine getNodeAffine()
      {
         return nodeAffine;
      }

      private ObjectProperty<Point3D> positionProperty = null;

      public ObjectProperty<Point3D> positionProperty()
      {
         if (positionProperty == null)
         {
            positionProperty = new SimpleObjectProperty<>(this, "position");
            positionProperty.addListener((observable, oldValue, newValue) ->
            {
               if (newValue == null)
                  return;
               if (newValue.equals(oldValue))
                  return;

               double x = newValue.getX();
               double y = newValue.getY();
               double z = newValue.getZ();
               nodeAffine.setTx(x);
               nodeAffine.setTy(y);
               nodeAffine.setTz(z);
            });
         }
         return positionProperty;
      }

      private ObjectProperty<Orientation3DBasics> orientationProperty = null;

      public ObjectProperty<Orientation3DBasics> orientationProperty()
      {
         if (orientationProperty == null)
         {
            orientationProperty = new SimpleObjectProperty<>(this, "orientation");
            orientationProperty.addListener((observable, oldValue, newValue) ->
            {
               if (newValue == null)
                  return;
               if (newValue.equals(oldValue))
                  return;

               RotationMatrix rotationMatrix = new RotationMatrix(newValue);
               nodeAffine.setToTransform(rotationMatrix.getM00(),
                                         rotationMatrix.getM01(),
                                         rotationMatrix.getM02(),
                                         nodeAffine.getTx(),
                                         rotationMatrix.getM10(),
                                         rotationMatrix.getM11(),
                                         rotationMatrix.getM12(),
                                         nodeAffine.getTy(),
                                         rotationMatrix.getM20(),
                                         rotationMatrix.getM21(),
                                         rotationMatrix.getM22(),
                                         nodeAffine.getTz());
            });
         }
         return orientationProperty;
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
