package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class FusedSuperPixelImageViewer
{
   private final AtomicReference<RawSuperPixelImage> lidarImageFusionDataToRender;

   protected final JavaFXMultiColorMeshBuilder meshBuilder;

   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>(null);
   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();
   private final AtomicReference<Boolean> clear = new AtomicReference<>(false);

   public FusedSuperPixelImageViewer(SharedMemoryJavaFXMessager messager)
   {
      lidarImageFusionDataToRender = messager.createInput(LidarImageFusionAPI.FusionDataState, null);

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      messager.registerTopicListener(LidarImageFusionAPI.ShowFusionData, (content) -> unpackFusionData());
   }

   private void unpackFusionData()
   {
      clear();
      double lineWidth = 0.01;
      meshBuilder.clear();
      RawSuperPixelImage rawSuperPixelImage = lidarImageFusionDataToRender.get();

      if (rawSuperPixelImage == null)
         return;

      int numberOfSegment = rawSuperPixelImage.getNumberOfImageSegments();

      List<PlanarRegionSegmentationRawData> planarRegionSegmentationRawDataList = new ArrayList<>();

      for (int i = 0; i < numberOfSegment; i++)
      {
         RawSuperPixelData data = rawSuperPixelImage.getFusionDataSegment(i);
         FusedSuperPixelData fusedSuperPixelData = new FusedSuperPixelData(data);

         PlanarRegionSegmentationRawData planarRegionSegmentationRawData = new PlanarRegionSegmentationRawData(i, fusedSuperPixelData.getNormal(),
                                                                                                               fusedSuperPixelData.getCenter(),
                                                                                                               fusedSuperPixelData.getPointsInSegment());

         planarRegionSegmentationRawDataList.add(planarRegionSegmentationRawData);
      }

      for (int i = 0; i < numberOfSegment; i++)
      {
         int randomID = new Random().nextInt();
         Color regionColor = getRegionColor(randomID);
         RawSuperPixelData data = rawSuperPixelImage.getFusionDataSegment(i);
         Point3D center = data.getCenter();
         Vector3D normal = data.getNormal();
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

      if (clear.getAndSet(false))
         children.clear();

      if (newScanMeshView != null)
      {
         children.add(newScanMeshView);
      }
   }

   public void clear()
   {
      clear.set(true);
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
