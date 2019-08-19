package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedXGaitStepStream extends QuadrupedStepStream<QuadrupedTeleopCommand>
{
   private static final int NUMBER_OF_PREVIEW_STEPS = 16;

   private final QuadrupedXGaitPlanner xGaitStepPlanner = new QuadrupedXGaitPlanner();
   private final YoQuadrupedXGaitSettings xGaitSettings;

   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps = new ArrayList<>();
   private final EndDependentList<QuadrupedTimedStep> currentSteps = new EndDependentList<>(QuadrupedTimedStep::new);
   private final YoFrameVector3D accumulatedStepAdjustment = new YoFrameVector3D("accumulatedStepAdjustment", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final double controlDT;
   private final DoubleProvider timestamp;
   private final YoDouble firstStepStartTime = new YoDouble("firstStepStartTime", registry);
   private final Point3D tempPoint = new Point3D();

   public QuadrupedXGaitStepStream(QuadrupedReferenceFrames referenceFrames, DoubleProvider timestamp, double controlDT, FrameVector3DReadOnly upcomingStepAdjustment, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                   YoVariableRegistry parentRegistry)
   {
      super("xgait", upcomingStepAdjustment, parentRegistry);
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }

      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.controlDT = controlDT;
      this.timestamp = timestamp;
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
      firstStepStartTime.set(timestamp.getValue() + initialTransferDurationForShifting.getValue());
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, desiredVelocity, initialQuadrant, supportCentroid, firstStepStartTime.getDoubleValue(), bodyYaw.getValue(), xGaitSettings);

      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = xGaitPreviewSteps.get(i).getRobotQuadrant().getEnd();
         currentSteps.get(robotEnd).set(xGaitPreviewSteps.get(i));
      }

      this.stepPlanIsAdjustable.set(teleopCommand.areStepsAdjustable());
      addStepsToSequence(stepSequence);
      accumulatedStepAdjustment.setToZero();
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
      if(timestamp.getValue() >= firstStepStartTime.getDoubleValue())
         bodyYaw.add(desiredVelocity.getZ() * controlDT);

      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         QuadrupedTimedStep step = xGaitPreviewSteps.get(i);
         if(step.getTimeInterval().getStartTime() < timestamp.getValue())
         {
            currentSteps.get(step.getRobotQuadrant().getEnd()).set(step);
         }
      }

      // update xgait preview steps
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, currentSteps, desiredVelocity, timestamp.getValue(), bodyYaw.getDoubleValue(), xGaitSettings);

      // handle shifting upcoming steps
      accumulatedStepAdjustment.add(upcomingStepAdjustment);
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         tempPoint.set(xGaitPreviewSteps.get(i).getGoalPosition());
         tempPoint.add(accumulatedStepAdjustment);
         xGaitPreviewSteps.get(i).setGoalPosition(tempPoint);
      }

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
         if(xGaitPreviewSteps.get(i).getTimeInterval().getEndTime() >= timestamp.getValue())
         {
            stepSequence.add().set(xGaitPreviewSteps.get(i));
         }
      }
   }
}
