package us.ihmc.footstepPlanning.simplePlanners;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class TurnWalkTurnPlanner implements FootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private static final String STRAIGHT_PATH_NAME = "Forward Path";
   private static final double STRAIGHT_STEP_LENGTH = 0.45; // For Steppr: 0.30;
   private static final double STRAIGHT_STEP_WIDTH = 0.3; // For Steppr: 0.35;
   private static final String REVERSE_PATH_NAME = "Reverse Path";
   private static final double REVERSE_ANGLE = Math.PI;
   private static final double REVERSE_STEP_LENGTH = 0.15;
   private static final double REVERSE_STEP_WIDTH = 0.3; // For Steppr: 0.35;
   private static final String RIGHT_SHUFFLE_PATH_NAME = "Right Shuffle Path";
   private static final String LEFT_SHUFFLE_PATH_NAME = "Left Shuffle Path";
   private static final double SHUFFLE_STEP_LENGTH = 0.25;  // For Steppr: 0.3;
   private static final double SHUFFLE_STEP_WIDTH = 0.21;
   private static final double LEFT_SHUFFLE_ANGLE = -Math.PI / 2;

   private final double epsilon = 10E-6;

   public static double maximumHipOpeningAngle = Math.toRadians(20.0);
   public static double maximumHipClosingAngle = Math.toRadians(0.0);
   public static double turningStepWidth = 0.3;

   private final FramePose2D initialStanceFootPose = new FramePose2D();
   private final FramePose2D goalPose = new FramePose2D();
   private RobotSide initialStanceSide;
   private RobotSide lastStepSide;
   private final Pose2dReferenceFrame stanceFootFrame = new Pose2dReferenceFrame("StanceFootFrame", ReferenceFrame.getWorldFrame());
   private final Pose2dReferenceFrame turningFrame = new Pose2dReferenceFrame("TurningFrame", ReferenceFrame.getWorldFrame());
   private double groundHeight = 0.0;

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      this.initialStanceFootPose.set(FlatGroundPlanningUtils.pose2dFormPose(stanceFootPose));
      this.initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      this.initialStanceSide = side;
      this.lastStepSide = side;
      this.groundHeight = stanceFootPose.getZ();
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
      this.goalPose.set(FlatGroundPlanningUtils.pose2dFormPose(goalPose));
      this.goalPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   @Override
   public FootstepPlanningResult plan()
   {
      stanceFootFrame.setPoseAndUpdate(initialStanceFootPose);

      FramePoint2D goalPoint = new FramePoint2D(goalPose.getPosition());

      ArrayList<FramePose2D> footstepList = new ArrayList<>();

      // turn
      Point2D robotOffsetFromStanceFoot = new Point2D(0.0, lastStepSide.negateIfLeftSide(turningStepWidth / 2.0));
      FramePose2D robotPose = new FramePose2D(stanceFootFrame, robotOffsetFromStanceFoot, 0.0);
      FramePose2D robotPoseInWorld = new FramePose2D(robotPose);
      robotPoseInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      addTurnInPlaceToFacePoint(footstepList, robotPoseInWorld, goalPoint);

      // walk
      FramePoint2D robotPosition = new FramePoint2D(robotPoseInWorld.getPosition());
      double distanceToTravel = robotPosition.distance(goalPoint);
      addStraightWalk(footstepList, robotPosition, distanceToTravel);

      // turn
      FramePose2D stanceFootPose = new FramePose2D(stanceFootFrame);
      stanceFootPose.changeFrame(goalPose.getReferenceFrame());
      double turningAngle = AngleTools.trimAngleMinusPiToPi(goalPose.getYaw() - stanceFootPose.getYaw());
      FramePoint2D pointToTurnAbout = new FramePoint2D(stanceFootFrame, new Point2D(0.0, lastStepSide.negateIfLeftSide(STRAIGHT_STEP_WIDTH / 2.0)));
      addTurnInPlace(footstepList, turningAngle, pointToTurnAbout);

      // square up
      addSquareUp(footstepList, pointToTurnAbout);

      footstepPlan.clear();
      RobotSide robotSide = initialStanceSide;
      for (FramePose2D footstepPose2d : footstepList)
      {
         robotSide = robotSide.getOppositeSide();
         footstepPlan.addFootstep(robotSide, FlatGroundPlanningUtils.poseFormPose2d(footstepPose2d, groundHeight));
      }
      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   private void addSquareUp(ArrayList<FramePose2D> footstepList, FramePoint2D robotPosition)
   {
      robotPosition.changeFrame(stanceFootFrame);
      if (Math.abs(robotPosition.getX()) > 0.001)
         throw new RuntimeException("Can not square up for given position.");

      robotPosition.changeFrame(stanceFootFrame);
      FramePose2D footstepPose = new FramePose2D(stanceFootFrame);
      footstepPose.setY(2.0*robotPosition.getY());

      if(lastStepSide.equals(RobotSide.LEFT) && footstepPose.getY() > 0)
         throw new RuntimeException("Left foot can not be placed on right side of right foot");


      if (lastStepSide.equals(RobotSide.RIGHT) && footstepPose.getY() < 0)
         throw new RuntimeException("Right foot can not be placed on left side of left foot");

      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepList.add(footstepPose);
      stanceFootFrame.setPoseAndUpdate(footstepPose);
      lastStepSide = lastStepSide.getOppositeSide();
   }

   private void addStraightWalk(ArrayList<FramePose2D> footstepList, FramePoint2D startingPoint, double distanceToTravel)
   {

      if(distanceToTravel<epsilon)
         return;

      double straightSteps = Math.ceil(distanceToTravel / STRAIGHT_STEP_LENGTH);
      double stepLength = distanceToTravel / straightSteps;
      FramePoint2D startingPointInWorld = new FramePoint2D(startingPoint);
      startingPointInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < straightSteps; i++)
      {
         startingPoint.setIncludingFrame(startingPointInWorld);
         startingPoint.changeFrame(stanceFootFrame);

         FramePose2D nextFootStep = new FramePose2D(stanceFootFrame);
         nextFootStep.setX(stepLength);
         nextFootStep.setY(startingPoint.getY() + lastStepSide.negateIfLeftSide(STRAIGHT_STEP_WIDTH / 2.0));


         if(lastStepSide.equals(RobotSide.LEFT) && nextFootStep.getY() > 0)
            throw new RuntimeException("Left foot can not be placed on right side of right foot");


         if (lastStepSide.equals(RobotSide.RIGHT) && nextFootStep.getY() < 0)
            throw new RuntimeException("Right foot can not be placed on left side of left foot");

         nextFootStep.changeFrame(ReferenceFrame.getWorldFrame());
         footstepList.add(nextFootStep);
         stanceFootFrame.setPoseAndUpdate(nextFootStep);
         lastStepSide = lastStepSide.getOppositeSide();
      }
   }

   private void addTurnInPlaceToFacePoint(ArrayList<FramePose2D> footstepList, FramePose2D robotPose, FramePoint2D goalPoint)
   {
      double turningAngle = AngleTools.calculateHeading(robotPose, goalPoint, -robotPose.getYaw(), 0.0);
      FramePoint2D pointToTurnAbout = new FramePoint2D(robotPose.getPosition());
      addTurnInPlace(footstepList, turningAngle, pointToTurnAbout);
   }

   private void addTurnInPlace(ArrayList<FramePose2D> footstepList, double turningAngle, FramePoint2D pointToTurnAbout)
   {
      if(Math.abs(turningAngle)<epsilon)
         return;

      FramePoint2D pointToTurnAboutInWorld = new FramePoint2D(pointToTurnAbout);
      pointToTurnAboutInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      pointToTurnAbout.changeFrame(stanceFootFrame);
      if (Math.abs(pointToTurnAbout.getX()) > 0.001)
         throw new RuntimeException("Can not turn in place around given point.");

      RobotSide sideToTurnTo = turningAngle >= 0.0 ? RobotSide.LEFT : RobotSide.RIGHT;

      double twoStepTurnAngle = maximumHipClosingAngle + maximumHipOpeningAngle;
      double requiredDoubleSteps = Math.abs(turningAngle / twoStepTurnAngle);

      double turningSteps = 2.0 * Math.ceil(requiredDoubleSteps);
      double maxTurningAngle = Math.ceil(requiredDoubleSteps) * twoStepTurnAngle;
      boolean firstStepClosing = sideToTurnTo.equals(lastStepSide);
      if (firstStepClosing)
      {
         if (Math.floor(requiredDoubleSteps) * twoStepTurnAngle + maximumHipClosingAngle >= Math.abs(turningAngle))
         {
            turningSteps--;
            maxTurningAngle -= maximumHipOpeningAngle;
         }
      }
      else
      {
         if (Math.floor(requiredDoubleSteps) * twoStepTurnAngle + maximumHipOpeningAngle >= Math.abs(turningAngle))
         {
            turningSteps--;
            maxTurningAngle -= maximumHipClosingAngle;
         }
      }
      double scaleTurningAngle = Math.abs(turningAngle) / maxTurningAngle;

      for (int i = 0; i < turningSteps; i++)
      {
         FramePose2D turningFramePose = new FramePose2D(stanceFootFrame);
         pointToTurnAbout.setIncludingFrame(pointToTurnAboutInWorld);
         pointToTurnAbout.changeFrame(stanceFootFrame);
         turningFramePose.setY(pointToTurnAbout.getY());

         if (sideToTurnTo.equals(lastStepSide))
         {
            turningFramePose.setYaw(sideToTurnTo.negateIfRightSide(maximumHipClosingAngle * scaleTurningAngle));
         }
         else
         {
            turningFramePose.setYaw(sideToTurnTo.negateIfRightSide(maximumHipOpeningAngle * scaleTurningAngle));
         }
         turningFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         turningFrame.setPoseAndUpdate(turningFramePose);

         FramePose2D nextFootstep = new FramePose2D(turningFrame);
         nextFootstep.setY(lastStepSide.negateIfLeftSide(turningStepWidth / 2.0));

         if(lastStepSide.equals(RobotSide.LEFT) && nextFootstep.getY() > 0)
            throw new RuntimeException("Left foot can not be placed on right side of right foot");


         if (lastStepSide.equals(RobotSide.RIGHT) && nextFootstep.getY() < 0)
            throw new RuntimeException("Right foot can not be placed on left side of left foot");

         nextFootstep.changeFrame(ReferenceFrame.getWorldFrame());


         footstepList.add(nextFootstep);
         stanceFootFrame.setPoseAndUpdate(nextFootstep);
         lastStepSide = lastStepSide.getOppositeSide();

      }
      pointToTurnAbout.setIncludingFrame(pointToTurnAboutInWorld);
      pointToTurnAbout.changeFrame(stanceFootFrame);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }

   @Override
   public void setTimeout(double timeout)
   {

   }

   @Override
   public double getPlanningDuration()
   {
      return -1;
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizonLength)
   {}

}
