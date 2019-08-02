package us.ihmc.robotEnvironmentAwareness.fusion.data;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;

import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

public class FusedSuperPixelImageBuilder implements Runnable
{
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final AtomicReference<Boolean> clear;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);
   private final AtomicReference<List<FusedSuperPixelData>> fusedSuperPixelDataReference;

   private static final double lineWidth = 0.01;

   public FusedSuperPixelImageBuilder(SharedMemoryJavaFXMessager messager)
   {
      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      clear = messager.createInput(LidarImageFusionAPI.ClearREA, false);

      fusedSuperPixelDataReference = messager.createInput(LidarImageFusionAPI.FusedSuperPixelData);
   }

   @Override
   public void run()
   {
      List<FusedSuperPixelData> fusedSuperPixelData = fusedSuperPixelDataReference.getAndSet(null);

      // Reset both clears by using only one pipe
      if (clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (fusedSuperPixelData == null)
         return;

      meshAndMaterialToRender.set(generateMeshAndMaterial(fusedSuperPixelData));
   }

   private Pair<Mesh, Material> generateMeshAndMaterial(List<FusedSuperPixelData> fusedSuperPixels)
   {
      if (fusedSuperPixels == null)
         return null;

      meshBuilder.clear();

      fusedSuperPixels.forEach(rawSuperPixelData -> addSuperPixelToMeshBuilder(meshBuilder, rawSuperPixelData));
      Mesh scanMeshView = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();

      return new Pair<>(scanMeshView, material);
   }

   private static void addSuperPixelToMeshBuilder(JavaFXMultiColorMeshBuilder meshBuilder, FusedSuperPixelData rawSuperPixelData)
   {
      // todo this needs to be faster
      Color regionColor = getRegionColor();
      Point3DReadOnly center = rawSuperPixelData.getCenter();
      Vector3DReadOnly normal = rawSuperPixelData.getNormal();

      Point3D centerEnd = new Point3D(normal);
      centerEnd.scaleAdd(0.1, rawSuperPixelData.getCenter());

      meshBuilder.addLine(center, centerEnd, lineWidth, regionColor);
      rawSuperPixelData.getPointsInPixel().forEach(point -> meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.02), point, regionColor));
   }

   private static Color getRegionColor()
   {
      java.awt.Color awtColor = new java.awt.Color(new Random().nextInt());
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   public boolean hasNewMeshAndMaterial()
   {
      return meshAndMaterialToRender.get() != null;
   }

   public Pair<Mesh, Material> pollMeshAndMaterial()
   {
      return meshAndMaterialToRender.getAndSet(null);
   }
}
