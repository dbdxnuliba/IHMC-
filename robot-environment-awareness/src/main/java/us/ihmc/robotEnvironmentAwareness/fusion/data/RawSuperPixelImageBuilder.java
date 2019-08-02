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

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

public class RawSuperPixelImageBuilder implements Runnable
{
   protected final JavaFXMultiColorMeshBuilder meshBuilder;

   private final AtomicReference<Boolean> clear;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);
   private final AtomicReference<RawSuperPixelImage> rawSuperPixelImageReference;

   private static final double lineWidth = 0.01;

   public RawSuperPixelImageBuilder(SharedMemoryJavaFXMessager messager)
   {
      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(2048));

      clear = messager.createInput(LidarImageFusionAPI.ClearREA, false);

      rawSuperPixelImageReference = messager.createInput(LidarImageFusionAPI.RawSuperPixelData);
   }

   @Override
   public void run()
   {
      RawSuperPixelImage rawSuperPixelImage = rawSuperPixelImageReference.getAndSet(null);

      // Reset both clears by using only one pipe
      if (clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (rawSuperPixelImage == null)
         return;

      meshAndMaterialToRender.set(generateMeshAndMaterial(rawSuperPixelImage));
   }

   private Pair<Mesh, Material> generateMeshAndMaterial(RawSuperPixelImage rawSuperPixelImage)
   {
      if (rawSuperPixelImage == null)
         return null;

      meshBuilder.clear();

      for (RawSuperPixelData superPixelData : rawSuperPixelImage.getSuperPixelData())
      {
         addSuperPixelToMeshBuilder(meshBuilder, superPixelData);
      }
      Mesh scanMeshView = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();

      return new Pair<>(scanMeshView, material);
   }

   private static void addSuperPixelToMeshBuilder(JavaFXMultiColorMeshBuilder meshBuilder, RawSuperPixelData rawSuperPixelData)
   {
      // todo this needs to be faster
      Color regionColor = getRegionColor(rawSuperPixelData.isSparse());
      Point3DReadOnly center = rawSuperPixelData.getCenter();
      Vector3DReadOnly normal = rawSuperPixelData.getNormal();

      Point3D centerEnd = new Point3D(normal);
      centerEnd.scaleAdd(0.1, rawSuperPixelData.getCenter());

      meshBuilder.addLine(center, centerEnd, lineWidth, regionColor);
      for (Point3DReadOnly point : rawSuperPixelData.getPointsInPixel())
      {
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.02), point, regionColor);
      }
   }

   private static Color getRegionColor(boolean isSparse)
   {
      if (isSparse)
         return Color.BLACK;

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
