package us.ihmc.commonWalkingControlModules.capturePoint.nummerical;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

public class SupportSeqence
{
   private static final int INITIAL_CAPACITY = 50;
   private static final double UNSET_TIME = -1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble supportSequenceStartTime = new YoDouble("SupportSequenceStartTime", registry);
   private final DoubleProvider time;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final RecyclingArrayList<ConvexPolygon2D> supportPolygons = new RecyclingArrayList<>(INITIAL_CAPACITY, ConvexPolygon2D.class);
   private final TDoubleArrayList supportDurations = new TDoubleArrayList(INITIAL_CAPACITY, UNSET_TIME);

   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
   private final SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());
   private final SideDependentList<ConvexPolygon2D> movingPolygons = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());

   private final List<YoDouble> polygonStartTimes = new ArrayList<>();
   private final List<ConvexPolygon2DBasics> vizPolygons = new ArrayList<>();

   public SupportSeqence(ConvexPolygon2DReadOnly defaultSupportPolygon, SideDependentList<? extends ReferenceFrame> soleFrames, DoubleProvider time,
                         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicRegistry)
   {
      if (graphicRegistry != null)
      {
         for (int i = 0; i < 10; i++)
         {
            YoFrameConvexPolygon2D yoPolygon = new YoFrameConvexPolygon2D("SupportPolygon" + i, ReferenceFrame.getWorldFrame(), 10, registry);
            graphicRegistry.registerArtifact(getClass().getSimpleName(), new YoArtifactPolygon("SupportPolygon" + i, yoPolygon, Color.GRAY, false));
            vizPolygons.add(yoPolygon);
            polygonStartTimes.add(new YoDouble("SupportPolygonStartTime" + i, registry));
         }
      }

      this.defaultSupportPolygon.set(defaultSupportPolygon);
      this.time = time;
      this.soleFrames = soleFrames;
      parentRegistry.addChild(registry);
      reset();
   }

   public void reset()
   {
      supportPolygons.clear();
      supportDurations.reset();
      initializeStance();
      supportSequenceStartTime.set(time.getValue());
   }

   public void initializeStance()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygons.get(robotSide).set(defaultSupportPolygon);
         footPolygons.get(robotSide).applyTransform(soleFrames.get(robotSide).getTransformToRoot(), false);
      }
   }

   public void initializeStance(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.footPolygons.get(robotSide).set(footPolygons.get(robotSide));
      }
   }

   public void appendCurrentStance(double duration)
   {
      appendStance(duration, footPolygons);
   }

   public void appendStance(double duration, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      ConvexPolygon2D supportPolygon = supportPolygons.add();
      supportDurations.add(duration);
      supportPolygon.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         supportPolygon.addVertices(footPolygons.get(robotSide));
      }
      supportPolygon.update();
   }

   public void setFromFootsteps(List<Footstep> footsteps, List<FootstepTiming> footstepTimings, double finalTransferDuration, boolean startingSwing)
   {
      reset();

      for (RobotSide robotSide : RobotSide.values)
      {
         movingPolygons.get(robotSide).set(footPolygons.get(robotSide));
      }

      for (int stepIndex = 0; stepIndex < footsteps.size(); stepIndex++)
      {
         FootstepTiming footstepTiming = footstepTimings.get(stepIndex);
         Footstep footstep = footsteps.get(stepIndex);
         RobotSide stepSide = footstep.getRobotSide();

         // Add transfer
         if (stepIndex > 0 || !startingSwing)
            appendStance(footstepTiming.getTransferTime(), movingPolygons);

         // Add swing support phase
         appendSupportPhase(movingPolygons.get(stepSide.getOppositeSide()), footstepTiming.getSwingTime());
         extractSupportPolygon(footstep, movingPolygons.get(stepSide), defaultSupportPolygon);
      }

      appendStance(finalTransferDuration, movingPolygons);

      updateViz();
   }

   private void updateViz()
   {
      int max = Math.min(vizPolygons.size(), supportPolygons.size());
      for (int i = 0; i < max; i++)
      {
         vizPolygons.get(i).set(supportPolygons.get(i));
         double lastStart = i == 0 ? 0.0 : polygonStartTimes.get(i - 1).getValue();
         polygonStartTimes.get(i).set(lastStart + supportDurations.get(i));
      }
      for (int i = max; i < vizPolygons.size(); i++)
      {
         vizPolygons.get(i).setToNaN();
         polygonStartTimes.get(i).set(UNSET_TIME);
      }
   }

   private static void extractSupportPolygon(Footstep footstep, ConvexPolygon2D newFootPolygon, ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         newFootPolygon.clear();
         for (int i = 0; i < predictedContactPoints.size(); i++)
            newFootPolygon.addVertex(predictedContactPoints.get(i));
         newFootPolygon.update();
      }
      else
      {
         newFootPolygon.set(defaultSupportPolygon);
      }

      // TODO: fix this up
      newFootPolygon.applyTransform(footstep.getSoleReferenceFrame().getTransformToRoot());
   }

   public void appendSupportPhase(ConvexPolygon2DReadOnly supportPolygon, double supportDuration)
   {
      supportPolygons.add().set(supportPolygon);
      supportDurations.add(supportDuration);
   }

   public List<? extends ConvexPolygon2DReadOnly> getSupportPolygons()
   {
      return supportPolygons;
   }

   public TDoubleList getSupportDurations()
   {
      return supportDurations;
   }

   public double getTimeInSequence()
   {
      return time.getValue() - supportSequenceStartTime.getValue();
   }

   public boolean isFirstSupportPhaseOver()
   {
      return getTimeInSequence() >= supportDurations.get(0);
   }

   public double getTimeInSegmentRemaining()
   {
      return supportDurations.get(0) - getTimeInSequence();
   }
}
