package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;

public class QuadrupedXGaitStepStream extends QuadrupedStepStream<QuadrupedTeleopCommand>
{
   private static final int NUMBER_OF_PREVIEW_STEPS = 16;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitPlanner xGaitStepPlanner = new QuadrupedXGaitPlanner();
   private final YoQuadrupedXGaitSettings xGaitSettings;

   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps = new ArrayList<>();
   private final EndDependentList<QuadrupedTimedStep> currentSteps = new EndDependentList<>(QuadrupedTimedStep::new);

   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final double controlDT;
   private final DoubleProvider timestamp;

   public QuadrupedXGaitStepStream(QuadrupedReferenceFrames referenceFrames, DoubleProvider timestamp, double controlDT, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                   YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }

      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.controlDT = controlDT;
      this.timestamp = timestamp;

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntryInternal(QuadrupedTeleopCommand teleopCommand, PreallocatedList<? extends QuadrupedTimedStep> stepSequence, DoubleProvider initialTransferDurationForShifting)
   {
      desiredVelocity.set(teleopCommand.getDesiredVelocity());
      xGaitSettings.set(teleopCommand.getXGaitSettings());

      // initialize body orientation
      bodyYaw.set(bodyZUpFrame.getTransformToWorldFrame().getRotation().getYaw());

      // initialize step queue
      supportCentroid.setToZero(supportFrame);
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      double initialTime = timestamp.getValue() + initialTransferDurationForShifting.getValue();
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, desiredVelocity, initialQuadrant, supportCentroid, initialTime, bodyYaw.getValue(), xGaitSettings);

      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = xGaitPreviewSteps.get(i).getRobotQuadrant().getEnd();
         currentSteps.get(robotEnd).set(xGaitPreviewSteps.get(i));
      }

      addStepsToSequence(stepSequence);
   }

   @Override
   public void doActionInternal(QuadrupedTeleopCommand teleopCommand, PreallocatedList<? extends QuadrupedTimedStep> stepSequence)
   {
      if(teleopCommand != null)
      {
         desiredVelocity.set(teleopCommand.getDesiredVelocity());
         xGaitSettings.set(teleopCommand.getXGaitSettings());
      }

      // update body orientation
      bodyYaw.add(desiredVelocity.getZ() * controlDT);

      // update xgait preview steps
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, currentSteps, desiredVelocity, timestamp.getValue(), bodyYaw.getDoubleValue(), xGaitSettings);

      // add steps to sequence
      addStepsToSequence(stepSequence);
   }

   private void addStepsToSequence(PreallocatedList<? extends QuadrupedTimedStep> stepSequence)
   {
      stepSequence.clear();
      for(RobotEnd end : RobotEnd.values)
      {
         if(currentSteps.get(end).getTimeInterval().getEndTime() >= timestamp.getValue())
         {
            stepSequence.add().set(currentSteps.get(end));
         }
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         stepSequence.add().set(xGaitPreviewSteps.get(i));
      }
   }
}
