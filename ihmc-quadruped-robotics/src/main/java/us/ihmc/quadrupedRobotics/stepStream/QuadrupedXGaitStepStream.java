package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedConstantVelocityBodyPathProvider;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;

public class QuadrupedXGaitStepStream implements QuadrupedStepStream
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final int NUMBER_OF_PREVIOUS_STEPS = 16;

   private final DoubleParameter firstStepDelay = new DoubleParameter("firstStepDelay", registry, 0.5);
   private final QuadrupedXGaitPlanner xGaitStepPlanner = new QuadrupedXGaitPlanner();
   private final YoQuadrupedXGaitSettings xGaitSettings;

   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final Quaternion bodyOrientation = new Quaternion();
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final EndDependentList<QuadrupedTimedStep> xGaitCurrentSteps = new EndDependentList<>(QuadrupedTimedStep::new);
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps = new ArrayList<>();
   private final PreallocatedList<YoQuadrupedTimedStep> stepSequence;
   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameVector3D desiredHeading = new YoFrameVector3D("desiredHeading", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble timestamp;
   private final double controlDT;

   public QuadrupedXGaitStepStream(QuadrupedReferenceFrames referenceFrames, YoDouble timestamp, double controlDT, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                   YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      for (int i = 0; i < NUMBER_OF_PREVIOUS_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }

      this.stepSequence = new PreallocatedList<>(YoQuadrupedTimedStep.class,
                                                 SupplierBuilder.indexedSupplier(i -> new YoQuadrupedTimedStep("previewStep" + i, registry)),
                                                 NUMBER_OF_PREVIOUS_STEPS + 2);

      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.timestamp = timestamp;
      this.controlDT = controlDT;

      parentRegistry.addChild(registry);
   }

   @Override
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return stepSequence;
   }

   @Override
   public void onTouchDown(RobotQuadrant quadrant)
   {

   }

   @Override
   public void onEntry()
   {
      // initialize body orientation
      bodyZUpFrame.getTransformToWorldFrame().getRotation(bodyOrientation);
      bodyYaw.set(bodyOrientation.getYaw());

      // initialize step queue
      supportCentroid.setToZero(supportFrame);
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      supportCentroid.setToZero(supportFrame);
      double initialTime = timestamp.getDoubleValue() + firstStepDelay.getValue();
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, desiredHeading, initialQuadrant, supportCentroid, initialTime, bodyYaw.getValue(), xGaitSettings);

      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = xGaitPreviewSteps.get(i).getRobotQuadrant().getEnd();
         xGaitCurrentSteps.get(robotEnd).set(xGaitPreviewSteps.get(i));
      }
   }

   @Override
   public void doAction()
   {
      double currentTime = timestamp.getDoubleValue();

      // update body orientation
      bodyYaw.add(desiredHeading.getZ() * controlDT);
      bodyOrientation.setYawPitchRoll(bodyYaw.getDoubleValue(), 0.0, 0.0);

      // update xgait current steps
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         QuadrupedTimedStep xGaitPreviewStep = xGaitPreviewSteps.get(i);
         if (xGaitPreviewStep.getTimeInterval().getStartTime() <= currentTime)
         {
            xGaitCurrentSteps.get(xGaitPreviewStep.getRobotQuadrant().getEnd()).set(xGaitPreviewStep);
         }
      }

      // update xgait preview steps
      double currentYaw = bodyYaw.getDoubleValue();
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, xGaitCurrentSteps, desiredHeading, currentTime, currentYaw, xGaitSettings);

      // update step sequence
      stepSequence.clear();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (xGaitCurrentSteps.get(robotEnd).getTimeInterval().getEndTime() >= currentTime)
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(xGaitCurrentSteps.get(robotEnd));
         }
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         if (xGaitPreviewSteps.get(i).getTimeInterval().getEndTime() >= currentTime)
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(xGaitPreviewSteps.get(i));
         }
      }
   }

   @Override
   public void onExit()
   {
   }
}
