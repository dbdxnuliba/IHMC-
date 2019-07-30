package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;

public class LidarImageFusionDataViewer
{
   protected final JavaFXMultiColorMeshBuilder meshBuilder;

   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>(null);
   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();

   private final AtomicBoolean showSolution = new AtomicBoolean(true);
   private final AtomicBoolean clearSolution = new AtomicBoolean(false);

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));


   public LidarImageFusionDataViewer(SharedMemoryJavaFXMessager messager)
   {
      messager.registerTopicListener(LidarImageFusionAPI.FusionDataState, lidarImageFusionData ->
            executorService.submit(() -> unpackFusionData(lidarImageFusionData)));

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      messager.registerTopicListener(LidarImageFusionAPI.ShowFusionData, this::handleShowSolution);
   }


   private void handleShowSolution(boolean show)
   {
      showSolution.set(show);
      if (!show)
         clearSolution.set(true);
   }

   private synchronized void unpackFusionData(LidarImageFusionData lidarImageFusionData)
   {
      clear();
      double lineWidth = 0.01;
      meshBuilder.clear();

      if (lidarImageFusionData == null)
         return;

      int numberOfSegment = lidarImageFusionData.getNumberOfImageSegments();

      List<PlanarRegionSegmentationRawData> planarRegionSegmentationRawDataList = new ArrayList<>();

      for (int i = 0; i < numberOfSegment; i++)
      {
         SegmentedImageRawData data = lidarImageFusionData.getFusionDataSegment(i);
         SegmentationNodeData segmentationNodeData = new SegmentationNodeData(data);

         PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(i, segmentationNodeData.getNormal(),
                                                                                                               segmentationNodeData.getCenter(),
                                                                                                               segmentationNodeData.getPointsInSegment());

         planarRegionSegmentationRawDataList.add(planarRegionSegmentationRawData);
      }

      for (int i = 0; i < numberOfSegment; i++)
      {
         int randomID = new Random().nextInt();
         Color regionColor = getRegionColor(randomID);
         SegmentedImageRawData data = lidarImageFusionData.getFusionDataSegment(i);
         Point3DReadOnly center = data.getCenter();
         Vector3DReadOnly normal = data.getNormal();
         Point3D centerEnd = new Point3D(normal);
         centerEnd.scaleAdd(0.1, center);
         if (data.isSparse())
         {
            regionColor = Color.rgb(0, 0, 0);
         }

         meshBuilder.addLine(center, centerEnd, lineWidth, regionColor);
         for (Point3D point : data.getPoints())
            meshBuilder.addTetrahedron(0.02, point, regionColor);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      meshToRender.set(scanMeshView);
      meshBuilder.clear();
   }

   public void render()
   {
      MeshView newScanMeshView = meshToRender.getAndSet(null);

      if (clearSolution.getAndSet(false))
      {
         children.clear();
      }

      if (newScanMeshView != null && showSolution.get())
      {
         children.add(newScanMeshView);
      }
   }

   public void clear()
   {
      clearSolution.set(true);
   }

   public Node getRoot()
   {
      return root;
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }
}
