package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class FeetControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SideDependentList<FootTrajectoryGenerator> feetTrajectoryGenerator = new SideDependentList<>();
   private final SideDependentList<FootController> feetControllers;
   private final SideDependentList<MovingReferenceFrame> soleFrames;
   private final SideDependentList<YoFramePose> desiredPoses = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> desiredFootVelocities = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> desiredFootAccelerations = new SideDependentList<>();
   private final SideDependentList<CurrentRigidBodyStateProvider> currentFootState = new SideDependentList<>();
   private List<ContactState> plannedContactStateList;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameVector3D tempAcceleration = new FrameVector3D();

   public FeetControlModule(SideDependentList<MovingReferenceFrame> soleFrames, SideDependentList<FootController> feetControllers, YoVariableRegistry registry,
                            YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = "FeetController";
      this.soleFrames = soleFrames;
      this.feetControllers = feetControllers;
      for (RobotSide side : RobotSide.values)
      {
         String footPrefix = namePrefix + "Desired" + side.getCamelCaseNameForMiddleOfExpression() + "Foot";
         currentFootState.put(side, new CurrentRigidBodyStateProvider(soleFrames.get(side)));
         feetTrajectoryGenerator.put(side, new FootTrajectoryGenerator(side, registry, graphicsListRegistry));
         YoFramePose desiredFootPose = new YoFramePose(footPrefix + "Pose", worldFrame, registry);
         desiredPoses.put(side, desiredFootPose);
         YoFrameVector desiredFootVelocity = new YoFrameVector(footPrefix + "Velocities", worldFrame, registry);
         desiredFootVelocities.put(side, desiredFootVelocity);
         YoFrameVector desiredFootAcceleration = new YoFrameVector(footPrefix + "Acceleration", worldFrame, registry);
         desiredFootAccelerations.put(side, desiredFootAcceleration);
      }
   }

   public void initialize(FootTrajectoryParameters parameters)
   {
      for (RobotSide side : RobotSide.values)
      {
         FootTrajectoryGenerator trajectoryGenerator = feetTrajectoryGenerator.get(side);
         trajectoryGenerator.initialize(parameters.getNominalFirstSegmentPercentageDuraion(), parameters.getNominalLastSegmentPercentageDuration(),
                                        parameters.getDefaultFinalVelocity(), parameters.getDefaultFinalAcceleration(),
                                        parameters.getNominalHeightAboveGround());
      }
   }

   public void submitContactStatePlan(List<ContactState> contactStateList)
   {
      this.plannedContactStateList = contactStateList;
   }

   public void doControl(double time)
   {
      for (RobotSide side : RobotSide.values)
      {
         FootTrajectoryGenerator footTrajectoryGenerator = feetTrajectoryGenerator.get(side);
         footTrajectoryGenerator.compute(time);
         if (isReplanningNeeded(side))
            updateFootTrajectory(time, side, footTrajectoryGenerator);
         desiredPoses.get(side).setPosition(footTrajectoryGenerator.getPosition());
         desiredFootVelocities.get(side).set(footTrajectoryGenerator.getVelocity());
         desiredFootAccelerations.get(side).set(footTrajectoryGenerator.getAcceleration());
      }
   }

   private void updateFootTrajectory(double time, RobotSide side, FootTrajectoryGenerator footTrajectoryGenerator)
   {
      PrintTools.debug("Updating foot trajectory " + side.toString());
      getInitialFootLocation(side, tempPosition, tempVelocity, tempAcceleration);
      footTrajectoryGenerator.setInitialConditions(tempPosition, tempVelocity, tempAcceleration);
      PrintTools.debug("Initial:" + tempPosition);
      double duration = getFinalFootLocationFromContactStatePlan(side, tempPosition);
      footTrajectoryGenerator.setFinalConditions(tempPosition);
      PrintTools.debug("Duration: " + duration + ", Final:" + tempPosition);
      footTrajectoryGenerator.setDuration(duration);
      footTrajectoryGenerator.updateTrajectory(time);
      footTrajectoryGenerator.compute(time);
   }

   private void getInitialFootLocation(RobotSide side, FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D accelertionToSet)
   {
      CurrentRigidBodyStateProvider stateProvider = currentFootState.get(side);
      stateProvider.getPosition(positionToSet);
      stateProvider.getLinearVelocity(accelertionToSet);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.changeFrame(worldFrame);
   }

   public FramePoint3DReadOnly getDesiredFootPosition(RobotSide side)
   {
      return desiredPoses.get(side).getPosition();
   }

   public FrameVector3DReadOnly getDesiredFootVelocity(RobotSide side)
   {
      return desiredFootVelocities.get(side);
   }

   public FrameVector3DReadOnly getDesiredFootAcceleration(RobotSide side)
   {
      return desiredFootAccelerations.get(side);
   }

   public boolean isReplanningNeeded(RobotSide sideToReplan)
   {
      ContactState currentState = plannedContactStateList.get(0);
      boolean isFootInSwing = !currentState.isFootInContact(sideToReplan);
      boolean isDone = feetTrajectoryGenerator.get(sideToReplan).isDone();
      return isFootInSwing && isDone;
   }

   private double getFinalFootLocationFromContactStatePlan(RobotSide side, FramePoint3D finalPositionToSet)
   {
      if (plannedContactStateList.size() == 0)
         throw new RuntimeException("No contact states provided for trajectory generation");
      // handle the special case
      if (plannedContactStateList.size() == 1)
      {
         ContactState firstContactState = plannedContactStateList.get(0);
         if (!firstContactState.isFootInContact(side))
            throw new RuntimeException("Cannot locate final " + side.getCamelCaseNameForStartOfExpression() + " foot location from contact state plan");
         firstContactState.getPosition(side, finalPositionToSet);
         return firstContactState.getDuration();
      }
      // Decide whether the foot should be held in place or moving 
      ContactState currentState = plannedContactStateList.get(0);

      boolean isFootInContact = currentState.isFootInContact(side);
      double duration = currentState.getDuration();
      // Ignoring the current contact state
      for (int contactStateIndex = 1; contactStateIndex < plannedContactStateList.size(); contactStateIndex++)
      {
         ContactState state = plannedContactStateList.get(contactStateIndex);
         if (isFootInContact != state.isFootInContact(side))
         {
            if (!isFootInContact)
               state.getPosition(side, finalPositionToSet);
            else
               currentState.getPosition(side, finalPositionToSet);
            return duration;
         }
         duration += state.getDuration();
      }
      if (isFootInContact)
      {
         currentState.getPosition(side, finalPositionToSet);
         return duration;
      }
      else
         throw new RuntimeException("Cannot locate final " + side.getCamelCaseNameForStartOfExpression() + " foot location from contact state plan");
   }
}