package us.ihmc.quadrupedUI.graphics;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import jassimp.Jassimp;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.graphics.JAssImpJavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

public class ManualStepPlanGraphic extends Group
{
   private static final double RADIUS = 0.02;
   private static final double zOffset = 0.01;
   private static final QuadrantDependentList<Color> solutionFootstepColors = new QuadrantDependentList<>(Color.BLUE, Color.ORANGE, Color.DARKBLUE, Color.DARKORANGE);

   private final MeshView meshView = new MeshView();
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private Mesh mesh;
   private Material material;

   private final Group pawGroup = new Group();
   private final Mesh pawMesh;
   private final AtomicReference<MeshView[]> paws = new AtomicReference<>(null);

   public ManualStepPlanGraphic()
   {
      getChildren().addAll(pawGroup);

      animationTimer.start();
      try
      {
         pawMesh = JAssImpJavaFXTools.getJavaFxMeshes("us/ihmc/quadrupedUI/model/paw.stl")[0].getMesh();
      }
      catch (URISyntaxException | IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsynchronously(QuadrupedTimedStepListMessage plan)
   {
      executorService.submit(() -> {
         LogTools.debug("Received pawstep plan containing {} steps", plan.getQuadrupedStepList().size());
         generateMeshes(plan);
      });
   }

   public void generateMeshes(QuadrupedTimedStepListMessage message)
   {
      meshBuilder.clear();

      QuadrantDependentList<Color> colors = solutionFootstepColors;

      FramePoint3D footPosition = new FramePoint3D();
      MeshView[] newPaws = new MeshView[message.getQuadrupedStepList().size()];

      for (int i = 0; i < message.getQuadrupedStepList().size(); i++)
      {
         QuadrupedTimedStepMessage footstep = message.getQuadrupedStepList().get(i);
         Color regionColor = colors.get(RobotQuadrant.fromByte(footstep.getQuadrupedStepMessage().getRobotQuadrant()));

         footPosition.set(footstep.getQuadrupedStepMessage().getGoalPosition());
         footPosition.addZ(zOffset);

         meshBuilder.addSphere(RADIUS, footPosition, regionColor);

         MeshView pawView = new MeshView(pawMesh);
         pawView.getTransforms().add(new Translate(footPosition.getX(), footPosition.getY(), footPosition.getZ()));
         pawView.getTransforms().add(new Scale(0.002, 0.002, 0.002));
         pawView.setMaterial(new PhongMaterial(regionColor));
         newPaws[i] = pawView;
      }

      paws.set(newPaws);

      generateAndQueueForJavaFXUpdate();
   }

   public void clear()
   {
      meshBuilder.clear();
      generateAndQueueForJavaFXUpdate();
   }

   private void generateAndQueueForJavaFXUpdate()
   {
      Mesh mesh = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();

      synchronized (this)
      {
         this.mesh = mesh;
         this.material = material;
      }
   }

   private void handle(long now)
   {
      MeshView[] newPaws = paws.getAndSet(null);
      if (newPaws != null)
      {
         pawGroup.getChildren().clear();
         pawGroup.getChildren().addAll(newPaws);
      }
      synchronized (this)
      {
         meshView.setMesh(mesh);
         meshView.setMaterial(material);
      }
   }

   public void stop()
   {
      executorService.shutdown();
      animationTimer.stop();
   }
}
