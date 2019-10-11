package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.DepthCloudMessage;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;

public class DepthCloudViewer extends AbstractSourceViewer<DepthCloudMessage>
{
   private final AtomicReference<Integer> sizeOfPointCloud;

   public DepthCloudViewer(Topic<DepthCloudMessage> messageState, REAUIMessager uiMessager)
   {
      super(messageState, uiMessager);
      sizeOfPointCloud = uiMessager.createInput(REAModuleAPI.UIStereoVisionSize, PointCloudAnchorPaneController.initialSizeOfPointCloud);
   }

   @Override
   public void render()
   {
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (clear.getAndSet(false))
         children.clear();

      if (!enable.get())
         return;

      if (newScanMeshView != null)
      {
         children.clear();
         children.add(newScanMeshView);
      }
   }

   @Override
   public void unpackPointCloud(DepthCloudMessage message)
   {
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

         int colorValue = 0;
         Color color = StereoVisionPointCloudViewer.intToColor(colorValue);

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
}