package us.ihmc.quadrupedRobotics.stepStream;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedCommunication.QuadrupedTeleopCommand;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;

public class QuadrupedXGaitStepStream extends QuadrupedStepStream<QuadrupedTeleopCommand>
{
   private static final int NUMBER_OF_PREVIEW_STEPS = 16;

   private final DoubleParameter firstStepDelay = new DoubleParameter("firstStepDelay", registry, 0.5);
   private final QuadrupedXGaitPlanner xGaitStepPlanner = new QuadrupedXGaitPlanner();
   private final YoQuadrupedXGaitSettings xGaitSettings;

   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps = new ArrayList<>();

   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final double controlDT;

   public QuadrupedXGaitStepStream(QuadrupedReferenceFrames referenceFrames, YoDouble timestamp, double controlDT, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                   YoVariableRegistry parentRegistry)
   {
      super("xgait_", timestamp);
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }

      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.controlDT = controlDT;

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntryInternal(QuadrupedTeleopCommand teleopCommand)
   {
      desiredVelocity.set(teleopCommand.getDesiredVelocity());
      xGaitSettings.set(teleopCommand.getXGaitSettings());

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
         currentSteps.get(robotEnd).set(xGaitPreviewSteps.get(i));
      }
   }

   @Override
   public void doActionInternal(QuadrupedTeleopCommand teleopCommand)
   {
      if(teleopCommand != null)
      {
         desiredVelocity.set(teleopCommand.getDesiredVelocity());
         xGaitSettings.set(teleopCommand.getXGaitSettings());
      }

      // update body orientation
      bodyYaw.add(desiredVelocity.getZ() * controlDT);

      // update xgait preview steps
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, currentSteps, desiredVelocity, timestamp.getDoubleValue(), bodyYaw.getDoubleValue(), xGaitSettings);

      // add steps to sequence
      stepSequence.clear();
      for(RobotEnd end : RobotEnd.values)
      {
         if(currentSteps.get(end).getTimeInterval().getEndTime() >= timestamp.getDoubleValue())
         {
            stepSequence.add().set(currentSteps.get(end));
         }
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         stepSequence.add().set(xGaitPreviewSteps.get(i));
      }
   }

   @Override
   int getPlanCapacity()
   {
      return NUMBER_OF_PREVIEW_STEPS + 2;
   }
}
