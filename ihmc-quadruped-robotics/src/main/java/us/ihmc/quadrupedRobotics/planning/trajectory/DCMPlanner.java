package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class DCMPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = false;
   private static final double POINT_SIZE = 0.005;

   private static final int STEP_SEQUENCE_CAPACITY = 50;

   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstantCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final FrameTrajectory3D dcmTransitionTrajectory;

   private final DoubleParameter initialTransitionDurationParameter = new DoubleParameter("initialTransitionDuration", registry, 0.5);

   private final QuadrupedTimedContactSequence timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private final YoDouble controllerTime;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final YoInteger numberOfStepsInPlanner = new YoInteger("numberOfStepsInPlanner", registry);
   private final YoFramePoint3D perfectCMPPosition = new YoFramePoint3D("perfectCMPPosition", worldFrame, registry);


   private final YoBoolean isStanding = new YoBoolean("isStanding", registry);

   private final ReferenceFrame supportFrame;
   private final YoFramePoint3D dcmPositionAtStartOfState = new YoFramePoint3D("dcmPositionAtStartOfState", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D dcmPositionAtEndOfTransition = new YoFramePoint3D("dcmPositionAtEndOfTransition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble timeAtStartOfState = new YoDouble("timeAtStartOfState", registry);
   private final FramePoint3D initialTransitionDCM = new FramePoint3D();
   private final FramePoint3D finalTransitionDCM = new FramePoint3D();
   private final FramePoint3D finalDCM = new FramePoint3D();

   private final FramePoint3D tempPoint = new FramePoint3D();

   private final boolean debug;

   public DCMPlanner(double gravity, double nominalHeight, YoDouble robotTimestamp, ReferenceFrame supportFrame,
                     QuadrantDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(gravity, nominalHeight, robotTimestamp, supportFrame, soleFrames, parentRegistry, yoGraphicsListRegistry, false);
   }

   public DCMPlanner(double gravity, double nominalHeight, YoDouble robotTimestamp, ReferenceFrame supportFrame,
                     QuadrantDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
                     boolean debug)

   {
      this.controllerTime = robotTimestamp;
      this.supportFrame = supportFrame;
      this.soleFrames = soleFrames;
      this.debug = debug;
      this.dcmTransitionTrajectory = new FrameTrajectory3D(6, supportFrame);
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, gravity, nominalHeight, registry);
      piecewiseConstantCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(2 * STEP_SEQUENCE_CAPACITY, registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      piecewiseConstantCopTrajectory.setupVisualizers(yoGraphicsList, artifactList, POINT_SIZE);
      dcmTrajectory.setupVisualizers(yoGraphicsList, artifactList, POINT_SIZE);

      YoGraphicPosition perfectCMPPositionViz = new YoGraphicPosition("Perfect CMP Position", perfectCMPPosition, 0.002, YoAppearance.BlueViolet());

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", perfectCMPPositionViz);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", perfectCMPPositionViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
      numberOfStepsInPlanner.set(0);
   }

   public void setCoMHeight(double comHeight)
   {
      this.comHeight.set(comHeight);
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
      numberOfStepsInPlanner.increment();
   }

   public void initializeForStanding()
   {
      isStanding.set(true);
      timedContactSequence.clear();
      piecewiseConstantCopTrajectory.resetVariables();
      dcmTrajectory.resetVariables();
   }

   public void initializeForStepping(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FramePoint3DReadOnly dcmPosition)
   {
      isStanding.set(false);

      double currentTime = controllerTime.getDoubleValue();
      boolean isCurrentPlanValid = stepSequence.get(numberOfStepsInPlanner.getIntegerValue() - 1).getTimeInterval().getEndTime() > currentTime;

      if (isCurrentPlanValid)
      {
         // compute dcm trajectory
         computeDcmTrajectory(currentContactStates);

         dcmPositionAtStartOfState.setMatchingFrame(dcmPosition);
         timeAtStartOfState.set(controllerTime.getDoubleValue());
         computeTransitionTrajectory();
      }
   }

   private void computeDcmTrajectory(QuadrantDependentList<YoEnum<ContactState>> currentContactStates)
   {
      // compute piecewise constant center of pressure plan
      double currentTime = controllerTime.getDoubleValue();
      timedContactSequence.update(stepSequence, soleFrames, currentContactStates, currentTime);
      piecewiseConstantCopTrajectory.initializeTrajectory(timedContactSequence);

      // compute dcm trajectory with final boundary constraint
      int numberOfIntervals = piecewiseConstantCopTrajectory.getNumberOfIntervals();
      tempPoint.setIncludingFrame(piecewiseConstantCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempPoint.add(0, 0, comHeight.getDoubleValue());

      dcmTrajectory.setComHeight(comHeight.getDoubleValue());
      dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getCopPositionsAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), tempPoint);
   }

   private void computeTransitionTrajectory()
   {
      double transitionEndTime = piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1);
      double transitionStartTime = Math.max(timeAtStartOfState.getDoubleValue(), transitionEndTime - initialTransitionDurationParameter.getValue());

      dcmTrajectory.computeTrajectory(transitionEndTime);
      dcmTrajectory.getPosition(finalTransitionDCM);

      initialTransitionDCM.setIncludingFrame(dcmPositionAtStartOfState);
      initialTransitionDCM.changeFrame(dcmTransitionTrajectory.getReferenceFrame());
      finalTransitionDCM.changeFrame(dcmTransitionTrajectory.getReferenceFrame());

      dcmPositionAtEndOfTransition.setMatchingFrame(finalTransitionDCM);
      dcmTransitionTrajectory.setQuinticWithZeroTerminalVelocityAndAcceleration(transitionStartTime, transitionEndTime, initialTransitionDCM, finalTransitionDCM);

      if (debug)
         runTransitionDebugChecks(transitionStartTime, transitionEndTime);
   }

   private void runTransitionDebugChecks(double transitionStartTime, double transitionEndTime)
   {
      if (finalTransitionDCM.containsNaN())
         throw new IllegalArgumentException("Final DCM at end of transition contains NaN.");
      if (dcmPositionAtStartOfState.containsNaN())
         throw new IllegalArgumentException("DCM Position at start of state contains NaN.");
      if (!Double.isFinite(transitionStartTime))
         throw new IllegalArgumentException("Transition start time is not valid.");
      if (!Double.isFinite(transitionEndTime))
         throw new IllegalArgumentException("Transition end time is not valid.");
      if (transitionStartTime > transitionEndTime)
         throw new IllegalArgumentException("Transition start time " + transitionStartTime + " is after the transition end time " + transitionEndTime +".");
      if (!dcmTransitionTrajectory.isValidTrajectory())
         throw new IllegalArgumentException("Transition trajectory is invalid.");
   }

   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();

   public void computeDcmSetpoints(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                   FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      if (isStanding.getBooleanValue())
      {
         // update desired dcm position
         desiredDCMPosition.setToZero(supportFrame);
         desiredDCMVelocity.setToZero(supportFrame);
      }
      else
      {
         computeDcmTrajectory(currentContactStates);

         double currentTime = controllerTime.getDoubleValue();
         dcmTrajectory.computeTrajectory(currentTime);
         if (currentTime <= dcmTransitionTrajectory.getFinalTime())
         {
            computeTransitionTrajectory();

            dcmTransitionTrajectory.compute(currentTime);
            dcmTransitionTrajectory.getFramePosition(desiredDCMPosition);
            dcmTransitionTrajectory.getFrameVelocity(desiredDCMVelocity);

         }
         else
         {
            dcmTrajectory.getPosition(desiredDCMPosition);
            dcmTrajectory.getVelocity(desiredDCMVelocity);
         }

         dcmTrajectory.getPositionAtEndOfSwing(finalDCM);
      }

      desiredDCMPosition.changeFrame(desiredDCMPositionToPack.getReferenceFrame());
      desiredDCMVelocity.changeFrame(desiredDCMVelocityToPack.getReferenceFrame());

      if (debug)
         runOutputDebugChecks();

      desiredDCMPositionToPack.set(desiredDCMPosition);
      desiredDCMVelocityToPack.set(desiredDCMVelocity);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, dcmTrajectory.getNaturalFrequency(), perfectCMPPosition);
   }

   private void runOutputDebugChecks()
   {
      if (desiredDCMPosition.containsNaN())
         throw new IllegalArgumentException("Desired DCM Position contains NaN.");
      if (desiredDCMVelocity.containsNaN())
         throw new IllegalArgumentException("Desired DCM Velocity contains NaN.");
   }

   public void getDCMAtEndOfTransition(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {
      finalDesiredDCMToPack.setMatchingFrame(finalTransitionDCM);
   }

   public void getFinalDCMPosition(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {
      finalDesiredDCMToPack.setMatchingFrame(finalDCM);
   }

   public void getPerfectCMPPosition(FramePoint2D perfectCMPPositionToPack)
   {
      perfectCMPPositionToPack.setIncludingFrame(perfectCMPPosition);
   }

   public double getFinalTime()
   {
      return dcmTransitionTrajectory.getFinalTime();
   }
}
