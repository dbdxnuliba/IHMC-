package us.ihmc.quadrupedRobotics.stepStream;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
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
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final EndDependentList<QuadrupedTimedStep> xGaitCurrentSteps = new EndDependentList<>(QuadrupedTimedStep::new);
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps = new ArrayList<>();
   private final PreallocatedList<QuadrupedTimedStep> stepSequence = new PreallocatedList<>(QuadrupedTimedStep.class, QuadrupedTimedStep::new, NUMBER_OF_PREVIOUS_STEPS + 2);

   private final YoBoolean areStepsAdjustable = new YoBoolean("areStepsAdjustable", registry);
   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<MutableBoolean> touchdownFlag = new QuadrantDependentList<>(MutableBoolean::new);
   private final YoDouble timestamp;
   private final double controlDT;
   private final YoBoolean isWalking = new YoBoolean("isWalking", registry);

   public QuadrupedXGaitStepStream(QuadrupedReferenceFrames referenceFrames, YoDouble timestamp, double controlDT, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                   YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      for (int i = 0; i < NUMBER_OF_PREVIOUS_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }

      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.timestamp = timestamp;
      this.controlDT = controlDT;
      desiredVelocity.setToNaN();

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
      touchdownFlag.get(quadrant).setTrue();
   }

   public void onLiftOff(RobotQuadrant quadrant)
   {
      touchdownFlag.get(quadrant).setFalse();
   }

   @Override
   public void onEntry()
   {
      // initialize body orientation
      bodyYaw.set(bodyZUpFrame.getTransformToWorldFrame().getRotation().getYaw());

      // initialize step queue
      supportCentroid.setToZero(supportFrame);
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      double initialTime = timestamp.getDoubleValue() + firstStepDelay.getValue();
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, desiredVelocity, initialQuadrant, supportCentroid, initialTime, bodyYaw.getValue(), xGaitSettings);

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
      bodyYaw.add(desiredVelocity.getZ() * controlDT);

      // extend duration of current steps that have delayed touchdown
      double delayTime = calculateDelayTime();
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         xGaitPreviewSteps.get(i).getTimeInterval().shiftInterval(delayTime);
      }

      // update current steps
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
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, xGaitCurrentSteps, desiredVelocity, currentTime, currentYaw, xGaitSettings);

      // update step sequence
      stepSequence.clear();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (xGaitCurrentSteps.get(robotEnd).getTimeInterval().getEndTime() >= currentTime)
         {
            stepSequence.add().set(xGaitCurrentSteps.get(robotEnd));
         }
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         if (xGaitPreviewSteps.get(i).getTimeInterval().getEndTime() >= currentTime)
         {
            stepSequence.add().set(xGaitPreviewSteps.get(i));
         }
      }
   }

   private double calculateDelayTime()
   {
      double previewStepDelayTime = 0.0;
      for (RobotEnd end : RobotEnd.values)
      {
         QuadrupedTimedStep currentStep = xGaitCurrentSteps.get(end);
         RobotQuadrant quadrant = currentStep.getRobotQuadrant();
         if (touchdownFlag.get(quadrant).isFalse() && currentStep.getTimeInterval().getEndTime() < timestamp.getDoubleValue())
         {
            double currentStepDelay = timestamp.getDoubleValue() - currentStep.getTimeInterval().getEndTime();
            currentStep.getTimeInterval().shiftInterval(currentStepDelay);
            previewStepDelayTime = Math.max(currentStepDelay, previewStepDelayTime);
         }
      }
      return previewStepDelayTime;
   }

   @Override
   public void onExit()
   {
      desiredVelocity.setToZero();
   }

   public void acceptTeleopCommand(QuadrupedTeleopCommand teleopCommand)
   {
      this.isWalking.set(teleopCommand.isWalkingRequested());
      this.desiredVelocity.set(teleopCommand.getDesiredVelocity());
      this.xGaitSettings.set(teleopCommand.getXGaitSettings());
      this.areStepsAdjustable.set(teleopCommand.areStepsAdjustable());
   }

   public boolean areStepsAdjustable()
   {
      return areStepsAdjustable.getValue();
   }

   public boolean isStepPlanAvailable()
   {
      return isWalking.getBooleanValue();
   }
}
