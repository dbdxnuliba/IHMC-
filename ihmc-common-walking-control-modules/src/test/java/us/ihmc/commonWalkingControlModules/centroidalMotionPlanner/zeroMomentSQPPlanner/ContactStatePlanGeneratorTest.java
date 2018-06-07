package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class ContactStatePlanGeneratorTest
{
   private double width = 0.1;
   private double ankleToToeX = 0.0725;
   private double ankleToToeY = 0.0225;
   private double ankleToMidX = 0.03625;
   private double ankleToMidY = 0.04;
   private double ankleToHeelX = 0.0175;
   private double ankleToHeelY = 0.02;

   private List<Point2D> generateDefaultFootSupportPolygonVertexList()
   {
      return Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY), new Point2D(ankleToMidX, -ankleToMidY),
                       new Point2D(-ankleToHeelX, -ankleToHeelY), new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                   .collect(Collectors.toList());
   }

   private List<Point2D> generateSimpleFootSupportPolygonVertexList()
   {
      return Stream.of(new Point2D(0.1, 0.2), new Point2D(0.1, -0.2), new Point2D(-0.1, -0.2), new Point2D(-0.1, 0.2)).collect(Collectors.toList());
   }

   private ConvexPolygon2D generateSimpleFootSupportPolygon()
   {
      return new ConvexPolygon2D(generateSimpleFootSupportPolygonVertexList());
   }

   @Test(timeout = 200)
   public void testContactStateSupportPolygonGeneration()
   {
      double epsilon = Epsilons.ONE_BILLIONTH;
      ConvexPolygon2D defaultFootPolygon = generateDefaultFootSupportPolygon();
      ContactState contactStateToTest = new ContactState();
      ContactStatePlanGenerator.setContactStatesFromPoses(contactStateToTest, 0.1, new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, width), 0.0),
                                          new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -width), 0.0), defaultFootPolygon,
                                          defaultFootPolygon);
      assertTrue(contactStateToTest.isSupported());
      ConvexPolygon2D polygonToTest = new ConvexPolygon2D();
      contactStateToTest.getSupportPolygon(RobotSide.LEFT, polygonToTest);
      assertEquals("Resultant poly is " + polygonToTest.toString(), 6, polygonToTest.getNumberOfVertices());
      FramePoint2D pointForTesting = new FramePoint2D();
      pointForTesting.set(-ankleToHeelX, ankleToHeelY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(0), epsilon);
      pointForTesting.set(ankleToMidX, +ankleToMidY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(1), epsilon);
      pointForTesting.set(ankleToToeX, ankleToToeY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(2), epsilon);
      pointForTesting.set(ankleToToeX, -ankleToToeY - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(3), epsilon);
      pointForTesting.set(ankleToMidX, -ankleToMidY - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(4), epsilon);
      pointForTesting.set(-ankleToHeelX, -ankleToHeelY - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(5), epsilon);

   }

   @Test(timeout = 200)
   public void testContactStateSupportPolygonGenerationWithPoseRotation()
   {
      double epsilon = Epsilons.ONE_BILLIONTH;
      ConvexPolygon2D defaultFootPolygon = generateDefaultFootSupportPolygon();
      ContactStatePlanGenerator generator = new ContactStatePlanGenerator();
      ContactState contactStateToTest = new ContactState();
      ContactStatePlanGenerator.setContactStatesFromPoses(contactStateToTest, 0.1, new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, width), 0.0),
                                          new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -width), 0.0), defaultFootPolygon,
                                          defaultFootPolygon);
      assertTrue(contactStateToTest.isSupported());
      ConvexPolygon2D polygonToTest = new ConvexPolygon2D();
      contactStateToTest.getSupportPolygon(RobotSide.LEFT, polygonToTest);
      assertEquals("Resultant poly is " + polygonToTest.toString(), 6, polygonToTest.getNumberOfVertices());
      FramePoint2D pointForTesting = new FramePoint2D();
      pointForTesting.set(-ankleToMidY - width, -ankleToMidX);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(0), epsilon);
      pointForTesting.set(-ankleToHeelY - width, ankleToHeelX);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(1), epsilon);
      pointForTesting.set(ankleToHeelY + width, ankleToHeelX);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(2), epsilon);
      pointForTesting.set(ankleToMidY + width, -ankleToMidX);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(3), epsilon);
      pointForTesting.set(ankleToToeY + width, -ankleToToeX);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(4), epsilon);
      pointForTesting.set(-ankleToToeY - width, -ankleToToeX);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, polygonToTest.getVertex(5), epsilon);
   }

   private ConvexPolygon2D generateDefaultFootSupportPolygon()
   {
      return new ConvexPolygon2D(generateDefaultFootSupportPolygonVertexList());
   }

   @Test(timeout = 200)
   public void testJumpPlanGeneration()
   {
      ConvexPolygon2D defaultSupportPolygonAnkleFrame = generateDefaultFootSupportPolygon();
      FramePose2D initialPelvisPose = new FramePose2D();
      Pose2D pelvisPoseChangePerJump = new Pose2D(0.1, 0.1, Math.PI / 4);
      int numberOfJumps = 2;
      ArrayList<ContactState> contactStates = new ArrayList<>();
      for (int i = 0; i < 2 * numberOfJumps + 1; i++)
         contactStates.add(new ContactState());
      Pose2D rightAnklePoseOffset = new Pose2D(0.0, -0.1, 0.0);
      Pose2D leftAnklePoseOffset = new Pose2D(0.0, 0.1, 0.0);
      ContactStatePlanGenerator contactStatePlanGenerationHelper = new ContactStatePlanGenerator();
      contactStatePlanGenerationHelper.generateContactStatePlanForJumping(contactStates, numberOfJumps, initialPelvisPose, pelvisPoseChangePerJump,
                                                                          leftAnklePoseOffset, rightAnklePoseOffset, 0.1, 0.3, defaultSupportPolygonAnkleFrame);
   }

   @Test(timeout = 1000)
   public void testWalkingPlanGenerationFromAlternatingSteps()
   {
      ContactStatePlanGenerator planGenerator = new ContactStatePlanGenerator();
      List<FramePose2D> footPoseList = new ArrayList<>();
      List<ContactState> contactStateList = new ArrayList<>();
      int numberOfSteps = 3;
      for (int i = 0; i < numberOfSteps + 2; i++)
         footPoseList.add(new FramePose2D());
      for (int i = 0; i < 2 * numberOfSteps + 5; i++)
         contactStateList.add(new ContactState());
      Vector2D stepSize = new Vector2D(0.1, 0.2);
      FramePose2D initialLeftAnklePose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, 0.15), 0.0);
      FramePose2D initialRightAnklePose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -0.15), 0.0);
      RobotSide startSide = RobotSide.LEFT;
      boolean endInDoubleSupport = true;
      ConvexPolygon2D defaultFootPolygon = generateDefaultFootSupportPolygon();
      planGenerator.generateAlternatingFootstepPoses(footPoseList, initialLeftAnklePose, initialRightAnklePose, stepSize, numberOfSteps, startSide,
                                                     endInDoubleSupport);
      ContactStatePlanGenerator.processFootstepPlanForWalking(footPoseList, contactStateList, defaultFootPolygon, defaultFootPolygon, startSide, true, endInDoubleSupport,
                                                  0.4, 0.2);
      for (int i = 0; i < contactStateList.size(); i++)
         PrintTools.debug(contactStateList.get(i).toString());
   }

   @Test(timeout = 1000)
   public void testRunningPlanGenerationFromAlternatingSteps()
   {
      ContactStatePlanGenerator planGenerator = new ContactStatePlanGenerator();
      List<FramePose2D> footPoseList = new ArrayList<>();
      List<ContactState> contactStateList = new ArrayList<>();
      int numberOfSteps = 3;
      for (int i = 0; i < numberOfSteps + 2; i++)
         footPoseList.add(new FramePose2D());
      for (int i = 0; i < 2 * numberOfSteps + 3; i++)
         contactStateList.add(new ContactState());
      Vector2D stepSize = new Vector2D(0.1, 0.2);
      FramePose2D initialLeftAnklePose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, 0.15), 0.0);
      FramePose2D initialRightAnklePose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -0.15), 0.0);
      RobotSide startSide = RobotSide.LEFT;
      ConvexPolygon2D defaultFootPolygon = generateDefaultFootSupportPolygon();
      planGenerator.generateAlternatingFootstepPoses(footPoseList, initialLeftAnklePose, initialRightAnklePose, stepSize, numberOfSteps, startSide,
                                                     true);
      ContactStatePlanGenerator.processFootstepPlanForRunning(footPoseList, contactStateList, defaultFootPolygon, defaultFootPolygon, startSide, 0.2, 0.4, 0.4, false, true, true, false);
      for (int i = 0; i < contactStateList.size(); i++)
         PrintTools.debug(contactStateList.get(i).toString());
   }
   
   @Test(timeout = 1000)
   public void testAlternatingFootstepGeneration()
   {
      ContactStatePlanGenerator planGenerator = new ContactStatePlanGenerator();
      List<FramePose2D> footPoseList = new ArrayList<>();
      int numberOfSteps = 3;
      for (int i = 0; i < numberOfSteps + 2; i++)
         footPoseList.add(new FramePose2D());
      Vector2D stepSize = new Vector2D(0.1, 0.2);
      FramePose2D initialLeftAnklePose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, 0.15), 0.0);
      FramePose2D initialRightAnklePose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -0.15), 0.0);
      planGenerator.generateAlternatingFootstepPoses(footPoseList, initialLeftAnklePose, initialRightAnklePose, stepSize, numberOfSteps, RobotSide.LEFT, true);
      for (int i = 0; i < footPoseList.size(); i++)
         PrintTools.debug(footPoseList.get(i).toString());
   }
}