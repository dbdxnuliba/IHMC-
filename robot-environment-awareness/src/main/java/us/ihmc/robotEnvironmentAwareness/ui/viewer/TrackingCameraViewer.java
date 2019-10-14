package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.TrackingCameraMessage;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class TrackingCameraViewer extends AnimationTimer
{
   private final JavaFXCoordinateSystem lidarCoordinateSystem;
   private final Affine sensorPose = new Affine();

   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();

   private final ArrayList<Point3D> sensorOriginHistory = new ArrayList<Point3D>();
   private final TDoubleArrayList confidenceFactorHistory = new TDoubleArrayList();

   private final Group root = new Group();
   private final Group affineRoot = new Group();
   private final Group historyRoot = new Group();

   protected static final float ORIGIN_POINT_SIZE = 0.01f;
   protected final JavaFXMultiColorMeshBuilder meshBuilder;

   public TrackingCameraViewer(REAUIMessager uiMessager)
   {
      uiMessager.registerTopicListener(REAModuleAPI.TrackingCameraClear, (c) -> clear());

      lidarCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      lidarCoordinateSystem.getTransforms().add(sensorPose);
      affineRoot.getChildren().add(lidarCoordinateSystem);
      affineRoot.setMouseTransparent(true);

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      root.getChildren().add(affineRoot);
      root.getChildren().add(historyRoot);

      uiMessager.registerTopicListener(REAModuleAPI.TrackingCameraState, this::handleMessage);
      uiMessager.registerModuleMessagerStateListener(isMessagerOpen -> {
         if (isMessagerOpen)
            start();
         else
            stop();
      });
   }

   private void clear()
   {
      sensorOriginHistory.clear();
      confidenceFactorHistory.clear();
   }

   @Override
   public void handle(long now)
   {
      Affine affine = lastAffine.getAndSet(null);
      if (affine != null)
         sensorPose.setToTransform(affine);

      historyRoot.getChildren().clear();
      meshBuilder.clear();
      Point3D32 point = new Point3D32();
      for (int i = 0; i < sensorOriginHistory.size(); i++)
      {
         int colorScaler = (int) (0xFF * confidenceFactorHistory.get(i));
         Color confidenceColor = Color.rgb(0xFF, colorScaler, 0);
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(ORIGIN_POINT_SIZE), point, confidenceColor);
      }
      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      historyRoot.getChildren().add(scanMeshView);
      meshBuilder.clear();
   }

   private void handleMessage(TrackingCameraMessage trackingCameraMessage)
   {
      if (trackingCameraMessage == null)
         return;
      Quaternion orientation = trackingCameraMessage.getSensorOrientation();
      Point3D position = trackingCameraMessage.getSensorPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
      sensorOriginHistory.add(new Point3D(position));
      confidenceFactorHistory.add(trackingCameraMessage.getQuality());
   }

   public Node getRoot()
   {
      return root;
   }
}
