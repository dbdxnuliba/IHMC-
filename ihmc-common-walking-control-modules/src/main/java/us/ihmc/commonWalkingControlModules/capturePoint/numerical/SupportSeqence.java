package us.ihmc.commonWalkingControlModules.capturePoint.numerical;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Precision;

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
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
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
   private final TDoubleArrayList supportInitialTimes = new TDoubleArrayList(INITIAL_CAPACITY, UNSET_TIME);

   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
   private final SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());

   private final SideDependentList<PoseReferenceFrame> movingSoleFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> movingPolygons = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());

   private final SideDependentList<RecyclingArrayList<ConvexPolygon2D>> footSupportSequences = new SideDependentList<>();
   private final SideDependentList<TDoubleArrayList> footSupportInitialTimes = new SideDependentList<>();

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

      for (RobotSide robotSide : RobotSide.values)
      {
         movingSoleFrames.put(robotSide, new PoseReferenceFrame(robotSide.getLowerCaseName() + "MovingSole", ReferenceFrame.getWorldFrame()));
         footSupportSequences.put(robotSide, new RecyclingArrayList<ConvexPolygon2D>(INITIAL_CAPACITY, ConvexPolygon2D.class));
         footSupportInitialTimes.put(robotSide, new TDoubleArrayList(INITIAL_CAPACITY, UNSET_TIME));
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
      supportInitialTimes.reset();
      for (RobotSide robotSide : RobotSide.values)
      {
         footSupportSequences.get(robotSide).clear();
         footSupportInitialTimes.get(robotSide).reset();
      }

      supportSequenceStartTime.set(time.getValue());
      initializeStance();
   }

   public void initializeStance()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footPolygons.get(robotSide).set(defaultSupportPolygon);
         footPolygons.get(robotSide).applyTransform(soleFrames.get(robotSide).getTransformToRoot(), false);
         movingPolygons.get(robotSide).set(footPolygons.get(robotSide));
         movingSoleFrames.get(robotSide).setPoseAndUpdate(soleFrames.get(robotSide).getTransformToRoot());
      }
   }

   public void setStance()
   {
      reset();

      ConvexPolygon2D supportPolygon = supportPolygons.add();
      supportInitialTimes.add(0.0);
      supportPolygon.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         supportPolygon.addVertices(footPolygons.get(robotSide));
      }
      supportPolygon.update();

      updateViz();
   }

   public void setFromFootsteps(List<Footstep> footsteps, List<FootstepTiming> footstepTimings, boolean startingSwing)
   {
      reset();

      // Add initial support states of the feet
      if (!footsteps.isEmpty())
      {
         RobotSide firstStepSide = footsteps.get(0).getRobotSide();
         if (startingSwing)
            footSupportSequences.get(firstStepSide).add().clearAndUpdate();
         else
            footSupportSequences.get(firstStepSide).add().set(movingPolygons.get(firstStepSide));
         footSupportSequences.get(firstStepSide.getOppositeSide()).add().set(movingPolygons.get(firstStepSide.getOppositeSide()));
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
            footSupportSequences.get(robotSide).add().set(movingPolygons.get(robotSide));
      }
      for (RobotSide robotSide : RobotSide.values)
         footSupportInitialTimes.get(robotSide).add(0.0);

      // Assemble the individual foot support trajectories
      for (int stepIndex = 0; stepIndex < footsteps.size(); stepIndex++)
      {
         FootstepTiming footstepTiming = footstepTimings.get(stepIndex);
         Footstep footstep = footsteps.get(stepIndex);
         RobotSide stepSide = footstep.getRobotSide();
         TDoubleArrayList swingFootInitialTimes = footSupportInitialTimes.get(stepSide);
         RecyclingArrayList<ConvexPolygon2D> swingFootSupports = footSupportSequences.get(stepSide);

         // Add swing - no support for foot
         if (!startingSwing || stepIndex > 0)
         {
            swingFootSupports.add().clearAndUpdate();
            double stepStartTime = Math.max(last(swingFootInitialTimes), last(footSupportInitialTimes.get(stepSide.getOppositeSide())));
            swingFootInitialTimes.add(stepStartTime + footstepTiming.getTransferTime());
         }

         // Update the moving polygon and sole frame to reflect that the step was taken.
         ConvexPolygon2D newFootPolygon = movingPolygons.get(stepSide);
         PoseReferenceFrame newSoleFrame = movingSoleFrames.get(stepSide);
         extractSupportPolygon(footstep, newFootPolygon, defaultSupportPolygon);
         newSoleFrame.setPoseAndUpdate(footstep.getFootstepPose());
         newFootPolygon.applyTransform(newSoleFrame.getTransformToRoot());

         // Add touchdown polygon
         swingFootSupports.add().set(newFootPolygon);
         swingFootInitialTimes.add(last(swingFootInitialTimes) + footstepTiming.getSwingTime());
      }

      // Convert the foot support trajectories to a full support trajectory
      int lIndex = 0;
      int rIndex = 0;
      ConvexPolygon2D lPolygon = footSupportSequences.get(RobotSide.LEFT).get(lIndex);
      ConvexPolygon2D rPolygon = footSupportSequences.get(RobotSide.RIGHT).get(rIndex);
      supportPolygons.add().set(lPolygon, rPolygon);
      supportInitialTimes.add(0.0);

      while (true)
      {
         double lNextTime;
         double rNextTime;
         if (footSupportInitialTimes.get(RobotSide.LEFT).size() == lIndex + 1)
            lNextTime = Double.POSITIVE_INFINITY;
         else
            lNextTime = footSupportInitialTimes.get(RobotSide.LEFT).get(lIndex + 1);
         if (footSupportInitialTimes.get(RobotSide.RIGHT).size() == rIndex + 1)
            rNextTime = Double.POSITIVE_INFINITY;
         else
            rNextTime = footSupportInitialTimes.get(RobotSide.RIGHT).get(rIndex + 1);

         if (Double.isInfinite(rNextTime) && Double.isInfinite(lNextTime))
            break;

         if (Precision.equals(lNextTime, rNextTime))
         {
            rIndex++;
            lIndex++;
            rPolygon = footSupportSequences.get(RobotSide.RIGHT).get(rIndex);
            lPolygon = footSupportSequences.get(RobotSide.LEFT).get(lIndex);
            supportInitialTimes.add(footSupportInitialTimes.get(RobotSide.LEFT).get(lIndex));
         }
         else if (lNextTime > rNextTime)
         {
            rIndex++;
            rPolygon = footSupportSequences.get(RobotSide.RIGHT).get(rIndex);
            supportInitialTimes.add(footSupportInitialTimes.get(RobotSide.RIGHT).get(rIndex));
         }
         else
         {
            lIndex++;
            lPolygon = footSupportSequences.get(RobotSide.LEFT).get(lIndex);
            supportInitialTimes.add(footSupportInitialTimes.get(RobotSide.LEFT).get(lIndex));
         }

         supportPolygons.add().set(lPolygon, rPolygon);
      }

      updateViz();
   }

   private static double last(TDoubleList list)
   {
      return list.get(list.size() - 1);
   }

   private void updateViz()
   {
      int max = Math.min(vizPolygons.size(), supportPolygons.size());
      for (int i = 0; i < max; i++)
      {
         vizPolygons.get(i).set(supportPolygons.get(i));
         double lastStart = i == 0 ? 0.0 : polygonStartTimes.get(i - 1).getValue();
         polygonStartTimes.get(i).set(lastStart + supportInitialTimes.get(i));
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
   }

   public void appendSupportPhase(ConvexPolygon2DReadOnly supportPolygon, double supportDuration)
   {
      supportPolygons.add().set(supportPolygon);
      supportInitialTimes.add(supportDuration);
   }

   public List<? extends ConvexPolygon2DReadOnly> getSupportPolygons()
   {
      return supportPolygons;
   }

   public TDoubleList getSupportTimes()
   {
      return supportInitialTimes;
   }

   public double getTimeInSequence()
   {
      return time.getValue() - supportSequenceStartTime.getValue();
   }

   public boolean isFirstSupportPhaseOver()
   {
      if (supportInitialTimes.size() == 1)
         return true;
      return getTimeInSequence() >= supportInitialTimes.get(1);
   }

   public double getTimeInSegmentRemaining()
   {
      if (supportInitialTimes.size() == 1)
         return 0.0;
      return supportInitialTimes.get(1) - getTimeInSequence();
   }
}
