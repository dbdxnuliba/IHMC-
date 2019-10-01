package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;

public class StereoVisionPointCloudViewer extends AbstractSourceViewer<StereoVisionPointCloudMessage>
{
   private final AtomicReference<Integer> sizeOfPointCloud;
   private final AtomicReference<Boolean> enablePreserve = new AtomicReference<Boolean>(false);

   private final double LENGTH_OF_SENSOR_FRAME = 0.3;
   private final JavaFXCoordinateSystem sensorCoordinateSystem;
   private final Affine sensorPose = new Affine();
   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();

   public StereoVisionPointCloudViewer(Topic<StereoVisionPointCloudMessage> messageState, REAUIMessager uiMessager)
   {
      super(messageState, uiMessager);
      sizeOfPointCloud = uiMessager.createInput(REAModuleAPI.UIStereoVisionSize, PointCloudAnchorPaneController.initialSizeOfPointCloud);
      sensorCoordinateSystem = new JavaFXCoordinateSystem(LENGTH_OF_SENSOR_FRAME);
      sensorCoordinateSystem.getTransforms().add(sensorPose);

      children.add(sensorCoordinateSystem);
   }

   public StereoVisionPointCloudViewer(Topic<StereoVisionPointCloudMessage> messageState, SharedMemoryMessager uiMessager)
   {
      super(messageState, uiMessager);
      sizeOfPointCloud = uiMessager.createInput(REAModuleAPI.UIStereoVisionSize, PointCloudAnchorPaneController.initialSizeOfPointCloud);
      sensorCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      sensorCoordinateSystem.getTransforms().add(sensorPose);

      children.add(sensorCoordinateSystem);
   }

   public void render()
   {
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (clear.getAndSet(false))
         children.clear();

      if (!enable.get())
         return;
      
      if (newScanMeshView != null)
      {
         if (!enablePreserve.get())
            children.clear();
         children.add(newScanMeshView);
         
         Affine affine = lastAffine.getAndSet(null);
         if (affine != null)
         {
            sensorPose.setToTransform(affine);
            JavaFXCoordinateSystem sensorCoordinateSystem = new JavaFXCoordinateSystem(LENGTH_OF_SENSOR_FRAME);
            sensorCoordinateSystem.getTransforms().add(affine);
            children.add(sensorCoordinateSystem);
         }
      }
   }

   @Override
   public void unpackPointCloud(StereoVisionPointCloudMessage message)
   {
      Quaternion orientation = message.getSensorOrientation();
      Point3D position = message.getSensorPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));

      Point3D32 scanPoint = new Point3D32();
      meshBuilder.clear();

      int numberOfScanPoints = message.getPointCloud().size() / 3;
      int sizeOfPointCloudToVisualize = Math.min(numberOfScanPoints, sizeOfPointCloud.get());

      Random random = new Random();
      for (int i = 0; i < sizeOfPointCloudToVisualize; i++)
      {
         int indexToVisualize;
         if (numberOfScanPoints < sizeOfPointCloud.get())
            indexToVisualize = i;
         else
            indexToVisualize = random.nextInt(numberOfScanPoints);

         int colorValue = message.getColors().get(indexToVisualize);
         Color color = intToColor(colorValue);

         MessageTools.unpackScanPoint(message, indexToVisualize, scanPoint);

         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   @Override
   protected Topic<Boolean> createEnableInput()
   {
      return REAModuleAPI.UIStereoVisionShow;
   }

   @Override
   protected Topic<Boolean> createClearInput()
   {
      return REAModuleAPI.UIStereoVisionClear;
   }

   public void enablePreserve(boolean enable)
   {
      enablePreserve.set(enable);
   }

   public static javafx.scene.paint.Color intToColor(int value)
   {
      int r = value >> 16 & 0xFF;
      int g = value >> 8 & 0xFF;
      int b = value >> 0 & 0xFF;
      return Color.rgb(r, g, b);
   }
}
