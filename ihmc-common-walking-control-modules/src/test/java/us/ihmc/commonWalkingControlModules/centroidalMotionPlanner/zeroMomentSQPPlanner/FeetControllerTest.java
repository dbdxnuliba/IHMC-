package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl.FeetControlModule;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl.FootTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.NumericalMovingReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private double width = 0.1;
   private double ankleToToeX = 0.0725;
   private double ankleToToeY = 0.0225;
   private double ankleToMidX = 0.03625;
   private double ankleToMidY = 0.04;
   private double ankleToHeelX = 0.0175;
   private double ankleToHeelY = 0.02;

   @Test(timeout = 1000)
   public void testDesiredGeneration()
   {
      FramePose2D leftFootPose2D = new FramePose2D(worldFrame, new Point2D(0.0, 0.15), 0.0);
      FramePose2D rightFootPose2D = new FramePose2D(worldFrame, new Point2D(0.0, -0.15), 0.0);
      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.set(leftFootPose2D);
      FramePose3D rightFootPose = new FramePose3D();
      rightFootPose.set(rightFootPose2D);
      PoseReferenceFrame leftSoleFrame = new PoseReferenceFrame("RightSoleFrame", leftFootPose);
      PoseReferenceFrame rightSoleFrame = new PoseReferenceFrame("LeftSoleFrame", rightFootPose);
      MovingReferenceFrame movingLeftSoleFrame = new NumericalMovingReferenceFrame(leftSoleFrame, 0.01);
      MovingReferenceFrame movingRightSoleFrame = new NumericalMovingReferenceFrame(rightSoleFrame, 0.01);
      movingLeftSoleFrame.update();
      movingRightSoleFrame.update();
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>(movingLeftSoleFrame, movingRightSoleFrame);
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      FeetControlModule controller = new FeetControlModule(soleFrames, null, registry, null);
      controller.initialize(new FootTrajectoryParameters());
      ContactStatePlanGenerator contactStatePlanner = new ContactStatePlanGenerator();
      int numberOfSteps = 2;
      List<ContactState> contactStatePlan = new ArrayList<>();
      for (int i = 0; i < 2 * numberOfSteps + 1; i++)
         contactStatePlan.add(new ContactState());
      Vector2D stepSize = new Vector2D(0.1, 0.0);
      ConvexPolygon2D defaultFootPolygon = generateDefaultFootSupportPolygon();
      double singleSupportDuration = 0.4;
      double doubleSupportDuration = 0.2;
      contactStatePlanner.generateContactStatePlanForWalking(contactStatePlan, leftFootPose2D, rightFootPose2D, stepSize, numberOfSteps, defaultFootPolygon,
                                                             defaultFootPolygon, RobotSide.LEFT, true, true, singleSupportDuration, doubleSupportDuration);
      for(int i = contactStatePlan.size() - 1; i > 0; i--)
         contactStatePlan.remove(i);
      controller.submitContactStatePlan(contactStatePlan);
      double totalMotionTime = numberOfSteps * (doubleSupportDuration + singleSupportDuration) + doubleSupportDuration;
      double dt = 0.01;
      //for(int i = 0; i < contactStatePlan.size(); i++)
      //   PrintTools.debug("Plan: " + i + contactStatePlan.get(i));
      for (double t = 0.0, stateStartTime = 0.0; t < totalMotionTime; t += dt)
      {
         controller.doControl(t);
         if (t - stateStartTime >= contactStatePlan.get(0).getDuration() && contactStatePlan.size() > 1)
         {
            stateStartTime = t;
            contactStatePlan.remove(0);
            PrintTools.debug("Switching state: " + contactStatePlan.get(0).toString());
            controller.submitContactStatePlan(contactStatePlan);
         }
         PrintTools.debug("Time: " + String.format("%.3f", t));
         for(RobotSide side : RobotSide.values)
         {
            PrintTools.debug(side.getCamelCaseNameForMiddleOfExpression() + ": Position: " + controller.getDesiredFootPosition(side).toString() + ", Velocity: "
                  + controller.getDesiredFootVelocity(side).toString() + ", Acceleration: " + controller.getDesiredFootAcceleration(side).toString());
         }
      }
   }

   private ConvexPolygon2D generateDefaultFootSupportPolygon()
   {
      return new ConvexPolygon2D(generateDefaultFootSupportPolygonVertexList());
   }

   private List<Point2D> generateDefaultFootSupportPolygonVertexList()
   {
      return Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY), new Point2D(ankleToMidX, -ankleToMidY),
                       new Point2D(-ankleToHeelX, -ankleToHeelY), new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                   .collect(Collectors.toList());
   }

}
