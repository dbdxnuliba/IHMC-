package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SideDependentList<FootTrajectoryGenerator> feetTrajectoryGenerator = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> soleFrames;
   private final SideDependentList<YoFramePose> desiredPoses = new SideDependentList<>();
   private final SideDependentList<CurrentRigidBodyStateProvider> currentFootState = new SideDependentList<>();
   private List<ContactState> plannedContactStateList;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameVector3D zeroAcceleration = new FrameVector3D();

   public FeetController(SideDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrames = soleFrames;
      for (RobotSide side : RobotSide.values)
      {
         currentFootState.put(side, new CurrentRigidBodyStateProvider(soleFrames.get(side)));
         feetTrajectoryGenerator.put(side, new FootTrajectoryGenerator(side, registry, graphicsListRegistry));
      }
   }

   public void initialize(FootTrajectoryParameters parameters)
   {
      for (RobotSide side : RobotSide.values)
      {
         FootTrajectoryGenerator trajectoryGenerator = feetTrajectoryGenerator.get(side);
         trajectoryGenerator.initialize(parameters.getNominalFirstSegmentPercentageDuraion(), parameters.getNominalLastSegmentPercentageDuration(),
                                        parameters.getDefaultFinalVelocity(), parameters.getDefaultFinalAcceleration(), parameters.getNominalHeightAboveGround());
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
         if (footTrajectoryGenerator.isDone())
            updateFootTrajectory(time, side, footTrajectoryGenerator);
         desiredPoses.get(side).setPosition(footTrajectoryGenerator.getPosition());
      }
   }

   private void updateFootTrajectory(double time, RobotSide side, FootTrajectoryGenerator footTrajectoryGenerator)
   {
      CurrentRigidBodyStateProvider stateProvider = currentFootState.get(side);
      stateProvider.getPosition(tempPosition);
      stateProvider.getLinearVelocity(tempVelocity);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.changeFrame(worldFrame);
      footTrajectoryGenerator.setInitialConditions(tempPosition, tempVelocity, zeroAcceleration);
      double duration = getFinalFootLocation(side, tempPosition);
      footTrajectoryGenerator.setFinalConditions(tempPosition);
      footTrajectoryGenerator.setDuration(duration);
      footTrajectoryGenerator.updateTrajectory(time);
      footTrajectoryGenerator.compute(time);
   }

   private double getFinalFootLocation(RobotSide side, FramePoint3D finalPositionToSet)
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
      double duration = plannedContactStateList.get(0).getDuration();
      // Ignoring the current contact state
      for (int contactStateIndex = 1; contactStateIndex < plannedContactStateList.size(); contactStateIndex++)
      {
         ContactState state = plannedContactStateList.get(contactStateIndex);
         if (state.isFootInContact(side))
         {
            state.getPosition(side, finalPositionToSet);
            return duration;
         }
         duration += state.getDuration();
      }
      throw new RuntimeException("Cannot locate final " + side.getCamelCaseNameForStartOfExpression() + " foot location from contact state plan");
   }
}
