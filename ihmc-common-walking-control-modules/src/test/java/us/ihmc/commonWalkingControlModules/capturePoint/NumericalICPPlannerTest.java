package us.ihmc.commonWalkingControlModules.capturePoint;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.function.ObjDoubleConsumer;

import javax.swing.JFrame;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.capturePoint.numerical.CopTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.numerical.NumericalICPPlanner;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.plotting.Plotter;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class NumericalICPPlannerTest
{
   private static final boolean visualizeInPlotter = true;
   private static final boolean visualizeInSCS = false;

   private static final double discretization = 0.02;
   private static final double previewTime = 3.0;
   private static final double omega = 3.0;
   private static final double adjustmentTime = 0.5;
   private static final int iterations = 100;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ArtifactList artifacts = new ArtifactList(NumericalICPPlannerTest.class.getSimpleName());

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static void packDSTest(Point2DBasics initialICP, List<ConvexPolygon2D> supportPolygons, TDoubleList supportTimes)
   {
      supportTimes.add(0.0);
      supportTimes.add(1.0);
      supportTimes.add(1.25);
      supportTimes.add(1.75);

      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(0.1, 0.2);
      polygon1.addVertex(-0.1, 0.2);
      polygon1.addVertex(0.1, -0.2);
      polygon1.addVertex(-0.1, -0.2);
      polygon1.update();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(0.1, 0.2);
      polygon2.addVertex(-0.1, 0.2);
      polygon2.update();
      ConvexPolygon2D polygon3 = new ConvexPolygon2D();
      polygon3.addVertex(0.1, 0.2);
      polygon3.addVertex(-0.1, 0.2);
      polygon3.addVertex(0.1 + 0.4, 0.2 + 0.5);
      polygon3.addVertex(-0.1 + 0.4, 0.2 + 0.5);
      polygon3.addVertex(0.1 + 0.4, -0.2 + 0.5);
      polygon3.addVertex(-0.1 + 0.4, -0.2 + 0.5);
      polygon3.update();
      ConvexPolygon2D polygon4 = new ConvexPolygon2D();
      polygon4.addVertex(0.1 + 0.4, 0.2 + 0.5);
      polygon4.addVertex(-0.1 + 0.4, 0.2 + 0.5);
      polygon4.addVertex(0.1 + 0.4, -0.2 + 0.5);
      polygon4.addVertex(-0.1 + 0.4, -0.2 + 0.5);
      polygon4.update();

      supportPolygons.add(polygon1);
      supportPolygons.add(polygon2);
      supportPolygons.add(polygon3);
      supportPolygons.add(polygon4);

      initialICP.set(supportPolygons.get(0).getCentroid());

      // Add some tracking error.
      initialICP.addX(0.05);
      initialICP.addY(-0.05);
   }

   private static void packSSTest(Point2DBasics initialICP, List<ConvexPolygon2D> supportPolygons, TDoubleList supportTimes)
   {
      supportTimes.add(0.0);
      supportTimes.add(0.5);
      supportTimes.add(0.6);
      supportTimes.add(1.1);
      supportTimes.add(1.2);

      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(0.1, 0.2);
      polygon1.addVertex(-0.1, 0.2);
      polygon1.addVertex(0.1, -0.2);
      polygon1.addVertex(-0.1, -0.2);
      polygon1.update();
      ConvexPolygon2D polygon3 = new ConvexPolygon2D(polygon1);
      polygon3.applyTransform(new RigidBodyTransform(new Quaternion(), new Vector3D(0.4, 0.5, 0.0)));
      ConvexPolygon2D polygon5 = new ConvexPolygon2D(polygon1);
      polygon5.applyTransform(new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 1.0, 0.0)));
      polygon5.update();

      ConvexPolygon2D polygon2 = new ConvexPolygon2D(polygon1, polygon3);
      ConvexPolygon2D polygon4 = new ConvexPolygon2D(polygon3, polygon5);

      supportPolygons.add(polygon1);
      supportPolygons.add(polygon2);
      supportPolygons.add(polygon3);
      supportPolygons.add(polygon4);
      supportPolygons.add(polygon5);

      initialICP.set(0.1, 0.3);
   }

   public NumericalICPPlannerTest(SimulationConstructionSet scs, YoGraphicsListRegistry graphicsListRegistry)
   {
      TDoubleList supportTimes = new TDoubleArrayList();
      List<ConvexPolygon2D> supportPolygons = new ArrayList<>();
      Point2DBasics initialIcp = new Point2D();
      Point2DBasics finalIcp = new Point2D();

      NumericalICPPlanner icpPlanner = new NumericalICPPlanner(discretization, previewTime, adjustmentTime);

      boolean useAm = false;
      ObjDoubleConsumer<Vector2DBasics> angularMomentumTrajectory = (am, t) -> {
         if (!useAm || t > 0.5)
            am.setToZero();
         else
            am.set(-0.1, 0.0);
      };

      packDSTest(initialIcp, supportPolygons, supportTimes);
      CopTrajectory copTrajectory = new CopTrajectory();
      copTrajectory.set(supportPolygons, supportTimes, 0.5);
      copTrajectory.accept(finalIcp, previewTime);

      long duration = 0;
      for (int i = 0; i < iterations; i++)
      {
         long startTime = System.nanoTime();
         icpPlanner.setOmega(omega);
         icpPlanner.setCopTrajectory(copTrajectory);
         icpPlanner.setAngularMomentumTrajectory(angularMomentumTrajectory);
         icpPlanner.setCopConstraints(supportPolygons, supportTimes);
         icpPlanner.setInitialIcp(initialIcp);
         icpPlanner.compute();
         duration = System.nanoTime() - startTime;
      }
      System.out.println("Took " + Conversions.nanosecondsToMilliseconds((double) (duration)) + " ms");

      setupGraphics(scs, graphicsListRegistry, previewTime, supportTimes, supportPolygons, initialIcp, finalIcp, copTrajectory, angularMomentumTrajectory,
                    icpPlanner);
   }

   private void setupGraphics(SimulationConstructionSet scs, YoGraphicsListRegistry graphicsListRegistry, double previewTime, TDoubleList supportDurations,
                              List<ConvexPolygon2D> supportPolygons, Point2DBasics initialIcp, Point2DBasics finalIcp, CopTrajectory copTrajectory,
                              ObjDoubleConsumer<Vector2DBasics> angularMomentumTrajectory, NumericalICPPlanner icpPlanner)
   {
      Point2DBasics cop = new Point2D();
      Point2DBasics icp = new Point2D();

      if (scs != null)
      {
         YoFrameConvexPolygon2D supportPolygon = new YoFrameConvexPolygon2D("supportPolygon", worldFrame, 10, registry);
         graphicsListRegistry.registerArtifact("Test", new YoArtifactPolygon("supportPolygon", supportPolygon, Color.GRAY, false));

         YoFramePoint2D referenceCop = new YoFramePoint2D("referenceCop", worldFrame, registry);
         graphicsListRegistry.registerArtifact("Test", new YoArtifactPosition("referenceCop", referenceCop, GraphicType.BALL, color(YoAppearance.DeepPink()), 0.005));
         YoFramePoint2D yoIcp = new YoFramePoint2D("Icp", worldFrame, registry);
         graphicsListRegistry.registerArtifact("Test", new YoArtifactPosition("Icp", yoIcp, GraphicType.BALL_WITH_ROTATED_CROSS, color(YoAppearance.Yellow()), 0.03));
         YoFramePoint2D yoCop = new YoFramePoint2D("Cop", worldFrame, registry);
         graphicsListRegistry.registerArtifact("Test", new YoArtifactPosition("Cop", yoCop, GraphicType.DIAMOND, color(YoAppearance.DarkViolet()), 0.01));
         YoFramePoint2D yoCmp = new YoFramePoint2D("Cmp", worldFrame, registry);
         graphicsListRegistry.registerArtifact("Test", new YoArtifactPosition("Cmp", yoCmp, GraphicType.BALL_WITH_CROSS, color(YoAppearance.Purple()), 0.01));

         YoFramePoint2D yoInitialIcp = new YoFramePoint2D("initialIcp", worldFrame, registry);
         graphicsListRegistry.registerArtifact("Test",
                                               new YoArtifactPosition("initialIcp", yoInitialIcp, GraphicType.BALL_WITH_ROTATED_CROSS, color(YoAppearance.Blue()), 0.03));
         yoInitialIcp.set(initialIcp);
         YoFramePoint2D yoFinalIcp = new YoFramePoint2D("finalIcp", worldFrame, registry);
         graphicsListRegistry.registerArtifact("Test", new YoArtifactPosition("finalIcp", yoFinalIcp, GraphicType.BALL_WITH_ROTATED_CROSS, color(YoAppearance.Beige()), 0.03));
         yoFinalIcp.set(finalIcp);

         scs.getRootRegistry().addChild(registry);
         scs.addYoGraphicsListRegistry(graphicsListRegistry);

         for (double t = 0.0; t < previewTime; t += 0.001)
         {
            int supportIndex = 0;
            while (supportIndex < supportDurations.size() - 1 && t > supportDurations.get(supportIndex + 1))
               supportIndex++;
            supportPolygon.set(supportPolygons.get(supportIndex));

            copTrajectory.accept(cop, t);
            referenceCop.set(cop);

            icpPlanner.getCop(t, cop);
            icpPlanner.getIcp(t, icp);
            yoCop.set(cop);
            yoIcp.set(icp);

            Vector2DBasics am = new Vector2D();
            angularMomentumTrajectory.accept(am, t);
            yoCmp.set(cop);
            yoCmp.addX(am.getY());
            yoCmp.subY(am.getX());

            scs.setTime(t);
            scs.tickAndUpdate();
         }
      }

      YoVariableRegistry artifactRagistry = new YoVariableRegistry("Artifacts");
      for (int i = 0; i < supportPolygons.size(); i++)
      {
         addToArtifacts("Support" + i, supportPolygons.get(i), Color.GRAY, artifactRagistry);
      }
      for (double t = 0.0; t < previewTime; t += discretization)
      {
         copTrajectory.accept(cop, t);
         addToArtifacts("referenceCop" + (int) (1000 * t), cop, GraphicType.BALL, Color.CYAN, 0.001, artifactRagistry);
         icpPlanner.getCop(t, cop);
         addToArtifacts("cop" + (int) (1000 * t), cop, GraphicType.BALL, Color.GREEN, 0.001, artifactRagistry);
         icpPlanner.getIcp(t, icp);
         addToArtifacts("icp" + (int) (1000 * t), icp, GraphicType.BALL, Color.BLUE, 0.001, artifactRagistry);
      }
      addToArtifacts("initialIcp", initialIcp, GraphicType.BALL_WITH_ROTATED_CROSS, Color.BLUE, 0.03, artifactRagistry);
      addToArtifacts("finalIcp", finalIcp, GraphicType.BALL_WITH_ROTATED_CROSS, Color.WHITE, 0.03, artifactRagistry);
   }

   private static Color color(AppearanceDefinition appearance)
   {
      YoAppearanceRGBColor yoAppearanceRGBColor = (YoAppearanceRGBColor) appearance;
      return new Color(yoAppearanceRGBColor.getRed(), yoAppearanceRGBColor.getGreen(), yoAppearanceRGBColor.getBlue());
   }

   public static void main(String[] args) throws InterruptedException
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      SimulationConstructionSet scs = visualizeInSCS ? new SimulationConstructionSet(new Robot("Test")) : null;

      new NumericalICPPlannerTest(scs, graphicsListRegistry);

      if (scs != null)
      {
         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.setPlotterName("Plotter");
         plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
         plotterFactory.createOverheadPlotter();
         scs.setPlaybackRealTimeRate(0.1);
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();
         scs.startOnAThread();
      }

      if (visualizeInPlotter)
         showPlotterAndSleep(artifacts);
   }

   private static void addToArtifacts(String name, Point2DReadOnly point, GraphicType graphicType, Color color, double size, YoVariableRegistry registry)
   {
      YoFramePoint2D yoPoint = new YoFramePoint2D(name, worldFrame, registry);
      artifacts.add(new YoArtifactPosition(name, yoPoint, graphicType, color, size));
      yoPoint.set(point);
   }

   private static void addToArtifacts(String name, ConvexPolygon2DReadOnly polygon, Color color, YoVariableRegistry registry)
   {
      YoFrameConvexPolygon2D yoPlanePolygon = new YoFrameConvexPolygon2D(name, worldFrame, 10, registry);
      artifacts.add(new YoArtifactPolygon(name, yoPlanePolygon, color, false));
      yoPlanePolygon.set(polygon);
   }

   private static void showPlotterAndSleep(ArtifactList artifacts) throws InterruptedException
   {
      Plotter plotter = new Plotter();
      plotter.setViewRange(2.0);
      artifacts.setVisible(true);
      JFrame frame = new JFrame(NumericalICPPlannerTest.class.getSimpleName() + " dt: " + discretization);
      Dimension preferredSize = new Dimension(600, 600);
      frame.setPreferredSize(preferredSize);
      frame.add(plotter.getJPanel(), BorderLayout.CENTER);
      frame.setSize(preferredSize);
      frame.setVisible(true);
      artifacts.addArtifactsToPlotter(plotter);

      CountDownLatch latch = new CountDownLatch(1);
      frame.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            latch.countDown();
         }
      });

      latch.await();
      frame.dispose();
   }
}
