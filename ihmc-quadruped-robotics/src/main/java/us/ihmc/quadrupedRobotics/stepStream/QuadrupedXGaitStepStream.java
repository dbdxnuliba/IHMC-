package us.ihmc.quadrupedRobotics.stepStream;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
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
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final EndDependentList<QuadrupedTimedStep> xGaitCurrentSteps = new EndDependentList<>(QuadrupedTimedStep::new);
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps = new ArrayList<>();
   private final PreallocatedList<QuadrupedTimedStep> stepSequence = new PreallocatedList<>(QuadrupedTimedStep.class, QuadrupedTimedStep::new, NUMBER_OF_PREVIOUS_STEPS + 2);

   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<MutableBoolean> touchdownFlag = new QuadrantDependentList<>(MutableBoolean::new);
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
      for (RobotEnd end : RobotEnd.values)
      {
         QuadrupedTimedStep currentStep = xGaitCurrentSteps.get(end);
         RobotQuadrant quadrant = currentStep.getRobotQuadrant();
         if (touchdownFlag.get(quadrant).isFalse() && currentStep.getTimeInterval().getEndTime() > timestamp.getDoubleValue())
         {
            currentStep.getTimeInterval().setEndTime(timestamp.getDoubleValue());
         }
      }

      // update xgait preview steps
      double currentYaw = bodyYaw.getDoubleValue();
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, xGaitCurrentSteps, desiredVelocity, currentTime, currentYaw, xGaitSettings);

      // update current steps
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         QuadrupedTimedStep xGaitPreviewStep = xGaitPreviewSteps.get(i);
         if (xGaitPreviewStep.getTimeInterval().getStartTime() <= currentTime)
         {
            xGaitCurrentSteps.get(xGaitPreviewStep.getRobotQuadrant().getEnd()).set(xGaitPreviewStep);
         }
      }

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

   @Override
   public void onExit()
   {
      desiredVelocity.setToNaN();
   }

   public void setDesiredVelocity(Tuple3DReadOnly desiredVelocity)
   {
      setDesiredVelocity(desiredVelocity.getX(), desiredVelocity.getY(), desiredVelocity.getZ());
   }

   public void setDesiredVelocity(double xVelocity, double yVelocity, double yawVelocity)
   {
      this.desiredVelocity.set(xVelocity, yVelocity, yawVelocity);
   }

   public boolean isStepPlanAvailable()
   {
      return !desiredVelocity.containsNaN();
   }
}
