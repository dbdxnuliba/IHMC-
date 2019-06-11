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
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.NonInvertibleTransformException;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.TransformChangedEvent;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
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
      messager.bindBidirectional(selectedPositionTopic, node.positionSelectedProperty(), false);
      messager.bindBidirectional(selectedOrientationTopic, node.orientationSelectedProperty(), false);
      messager.registerJavaFXSyncedTopicListener(selectedPositionTopic, m ->
      {
         if (!node.isShown())
            node.show(true);
      });
      messager.registerJavaFXSyncedTopicListener(selectedOrientationTopic, m ->
      {
         if (!node.isShown())
            node.show(true);
      });
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
      private final MeshView arrow = createArrowGraphic(DEFAULT_ARROW_LENGTH, 0.2 * DEFAULT_SIZE);
      private final Rotate arrowAdjustmentRotate = new Rotate();

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

         arrow.getTransforms().add(arrowAdjustmentRotate);
         getTransforms().add(nodeAffine);
         nodeAffine.addEventHandler(TransformChangedEvent.TRANSFORM_CHANGED, e -> adjustFeetPosition());
         nodeAffine.addEventHandler(TransformChangedEvent.TRANSFORM_CHANGED, e -> adjustArrowOrientation());

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

         setPositionMaterial(unselectedMaterial);
         setOrientationMaterial(unselectedMaterial);

         center.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> positionSelectedProperty.set(true));
         arrow.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> orientationSelectedProperty.set(true));
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

      private void adjustArrowOrientation()
      {
         javafx.geometry.Point3D localToParent = arrow.localToScene(DEFAULT_ARROW_LENGTH, 0.0, 0.0);
         Point3D arrowHeadPosition = new Point3D(localToParent.getX(), localToParent.getY(), localToParent.getZ());
         Point3DReadOnly adjustedPosition = footPositionAdjustment.apply(arrowHeadPosition);

         if (adjustedPosition == null)
         {
            arrowAdjustmentRotate.setAngle(0.0);
         }
         else
         {
            Vector3D adjustmentVector = new Vector3D();
            adjustmentVector.sub(adjustedPosition, arrowHeadPosition);

            try
            {
               javafx.geometry.Point3D localOffset = arrow.getLocalToSceneTransform()
                                                          .inverseDeltaTransform(adjustmentVector.getX(), adjustmentVector.getY(), adjustmentVector.getZ());
               Vector3D originalArrowVector = new Vector3D(DEFAULT_ARROW_LENGTH, 0.0, 0.0);
               Vector3D adjustedArrowVector = new Vector3D(localOffset.getX(), localOffset.getY(), localOffset.getZ());
               adjustedArrowVector.add(originalArrowVector);
               AxisAngle axisAngle = new AxisAngle();
               EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(originalArrowVector, adjustedArrowVector, axisAngle);
               appendOrientation(axisAngle, arrowAdjustmentRotate);
            }
            catch (NonInvertibleTransformException e)
            {
               e.printStackTrace();
               arrowAdjustmentRotate.setAngle(0.0);
            }
         }
      }

      private static void appendOrientation(Orientation3DReadOnly orientationToAdd, Rotate rotateToModify)
      {
         AxisAngle axisAngle = new AxisAngle();
         axisAngle.setAxisAngle(rotateToModify.getAxis().getX(),
                                rotateToModify.getAxis().getY(),
                                rotateToModify.getAxis().getZ(),
                                Math.toRadians(rotateToModify.getAngle()));
         axisAngle.append(orientationToAdd);
         rotateToModify.setAngle(Math.toDegrees(axisAngle.getAngle()));
         rotateToModify.setAxis(new javafx.geometry.Point3D(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ()));
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
         setUnselectedMaterial(unselectedMaterial);
         setSelectedMaterial(selectedMaterial);
      }

      public void setSelectedMaterial(Material selectedMaterial)
      {
         this.selectedMaterial = selectedMaterial;
         if (positionSelectedProperty.get())
            setPositionMaterial(selectedMaterial);
         if (orientationSelectedProperty.get())
            setOrientationMaterial(selectedMaterial);
      }

      public void setUnselectedMaterial(Material unselectedMaterial)
      {
         this.unselectedMaterial = unselectedMaterial;
         if (!positionSelectedProperty.get())
            setPositionMaterial(unselectedMaterial);
         if (!orientationSelectedProperty.get())
            setOrientationMaterial(unselectedMaterial);
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

               nodeAffine.setToTransform(nodeAffine.getMxx(),
                                         nodeAffine.getMxy(),
                                         nodeAffine.getMxz(),
                                         newValue.getX(),
                                         nodeAffine.getMyx(),
                                         nodeAffine.getMyy(),
                                         nodeAffine.getMyz(),
                                         newValue.getY(),
                                         nodeAffine.getMzx(),
                                         nodeAffine.getMzy(),
                                         nodeAffine.getMzz(),
                                         newValue.getZ());
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
