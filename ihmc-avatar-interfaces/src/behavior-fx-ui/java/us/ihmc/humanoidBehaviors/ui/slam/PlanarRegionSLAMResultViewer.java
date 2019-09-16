package us.ihmc.humanoidBehaviors.ui.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class PlanarRegionSLAMResultViewer
{
   private static final int NUMBER_OF_FRAMES = 100;
   private static final double LENGTH_OF_THE_LATEST_FRAME = 0.15;
   private static final double LENGTH_OF_THE_OLDEST_FRAME = 0.05;

   private final Messager messager;
   private final Group root = new Group();
   private final ObservableList<Node> children = root.getChildren();

   private final AnimationTimer renderMeshAnimation;

   // Most left one is the oldest one.
   private final List<RigidBodyTransform> transforms = new ArrayList<>();
   private final AtomicReference<Boolean> showResult;
   private final AtomicReference<RigidBodyTransform> incomingTransform;

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   public PlanarRegionSLAMResultViewer(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      showResult = messager.createInput(RealTimePlanarRegionSLAMAPI.ShowSLAMResult, false);
      incomingTransform = messager.createInput(RealTimePlanarRegionSLAMAPI.LatestSLAMTransform, null);

      TextureColorPalette1D colorPalette = new TextureColorPalette1D();
      colorPalette.setHueBased(1.0, 1.0);
      meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            update();
         }
      };
      renderMeshAnimation.start();
   }

   private void updateLatestTrasnfrom(RigidBodyTransform transform)
   {
      transforms.add(transform);
      if (transforms.size() >= NUMBER_OF_FRAMES)
      {
         transforms.remove(0);
      }

      if (!showResult.get())
         return;

      double maxLength = LENGTH_OF_THE_LATEST_FRAME;
      double minLength = maxLength - (maxLength - LENGTH_OF_THE_OLDEST_FRAME) * (transforms.size() - 1) / (NUMBER_OF_FRAMES - 1);
      clear();

      JavaFXCoordinateSystem referenceFrameCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      Quaternion orientation = new Quaternion(transforms.get(transforms.size() - 1).getRotation());
      Vector3DBasics translation = transforms.get(transforms.size() - 1).getTranslation();
      Affine affine = JavaFXTools.createAffineFromOrientation3DAndTuple(orientation, translation);
      referenceFrameCoordinateSystem.getTransforms().add(affine);
      children.add(referenceFrameCoordinateSystem);
      for(int i=0;i<transforms.size(); i++)
      {
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.01), transforms.get(i).getTranslation(), Color.rgb(255, 0, 0));
      }
      MeshView translations = new MeshView(meshBuilder.generateMesh());
      translations.setMaterial(meshBuilder.generateMaterial());
      children.addAll(translations);
      meshBuilder.clear();

      //      for (int i = 0; i < transforms.size(); i++)
      //      {
      //         double coordinateLength = minLength + (maxLength - minLength) * i / (transforms.size() - 1);
      //         JavaFXCoordinateSystem referenceFrameCoordinateSystem = new JavaFXCoordinateSystem(coordinateLength);
      //
      //         Quaternion orientation = new Quaternion(transforms.get(i).getRotation());
      //         Vector3DBasics translation = transforms.get(i).getTranslation();
      //         
      //         Affine affine = JavaFXTools.createAffineFromOrientation3DAndTuple(orientation, translation);
      //         
      //         referenceFrameCoordinateSystem.getTransforms().add(affine);
      //         children.add(referenceFrameCoordinateSystem);
      //      }
   }

   private void addTransformMesh()
   {

   }

   private void update()
   {
      if (incomingTransform.get() != null)
      {
         RigidBodyTransform latestTransform = incomingTransform.getAndSet(null);
         updateLatestTrasnfrom(latestTransform);
      }
   }

   private void clear()
   {
      children.clear();
   }

   public Node getRoot()
   {
      return root;
   }
}
