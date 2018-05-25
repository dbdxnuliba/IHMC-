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
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class ContactStatePlanGeneratorTest
{
   private double width = 0.1;
   private double ankleToToeX = 0.0725;
   private double ankleToToeY = 0.0225;
   private double ankleToMidX = 0.03625;
   private double ankleToMidY = 0.04;
   private double ankleToHeelX = 0.0175;
   private double ankleToHeelY = 0.02;

   @Test(timeout = 200)
   public void testMinimalVertexPolygonGeneration()
   {
      double epsilon = Epsilons.ONE_BILLIONTH;
      ContactStatePlanGenerator generator = new ContactStatePlanGenerator(12);
      ConvexPolygon2D problemPoly = new ConvexPolygon2D();
      List<Point2D> defaultFootPolygonPointsInAnkleFrame = generateDefaultFootSupportPolygonVertexList();
      ArrayList<Point2D> vertexList = new ArrayList<>();
      for (int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         Point2D leftFootPoint = new Point2D();
         leftFootPoint.set(defaultFootPolygonPointsInAnkleFrame.get(i).getX(), defaultFootPolygonPointsInAnkleFrame.get(i).getY() + width);
         Point2D rightFootPoint = new Point2D();
         rightFootPoint.set(defaultFootPolygonPointsInAnkleFrame.get(i).getX(), defaultFootPolygonPointsInAnkleFrame.get(i).getY() - width);
         vertexList.add(leftFootPoint);
         vertexList.add(rightFootPoint);
      }
      vertexList.add(new Point2D());
      vertexList.add(new Point2D());
      generator.generateMinimalVertexSupportPolygon(problemPoly, vertexList, defaultFootPolygonPointsInAnkleFrame.size() * 2);
      assertEquals("Resultant poly is " + problemPoly.toString(), 6, problemPoly.getNumberOfVertices());
      FramePoint2D pointForTesting = new FramePoint2D();
      pointForTesting.set(-ankleToHeelX, ankleToHeelY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(0), epsilon);
      pointForTesting.set(ankleToMidX, +ankleToMidY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(1), epsilon);
      pointForTesting.set(ankleToToeX, ankleToToeY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(2), epsilon);
      pointForTesting.set(ankleToToeX, -ankleToToeY - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(3), epsilon);
      pointForTesting.set(ankleToMidX, -ankleToMidY - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(4), epsilon);
      pointForTesting.set(-ankleToHeelX, -ankleToHeelY - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(5), epsilon);
   }

   @Test(timeout = 200)
   public void testRotatedMinimalVertexPolygonGeneration()
   {
      double epsilon = Epsilons.ONE_BILLIONTH;
      ContactStatePlanGenerator generator = new ContactStatePlanGenerator(12, 1e-5);
      ConvexPolygon2D problemPoly = new ConvexPolygon2D();
      List<Point2D> defaultFootPolygonPointsInAnkleFrame = generateDefaultFootSupportPolygonVertexList();
      ArrayList<Point2D> vertexList = new ArrayList<>();
      for (int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         Point2D leftFootPoint = new Point2D();
         leftFootPoint.set(defaultFootPolygonPointsInAnkleFrame.get(i).getX(), defaultFootPolygonPointsInAnkleFrame.get(i).getY() + width);
         Point2D rightFootPoint = new Point2D();
         rightFootPoint.set(defaultFootPolygonPointsInAnkleFrame.get(i).getY(), -defaultFootPolygonPointsInAnkleFrame.get(i).getX() - width);
         vertexList.add(leftFootPoint);
         vertexList.add(rightFootPoint);
      }
      vertexList.add(new Point2D());
      vertexList.add(new Point2D());
      generator.generateMinimalVertexSupportPolygon(problemPoly, vertexList, defaultFootPolygonPointsInAnkleFrame.size() * 2);
      assertEquals("Resultant poly is " + problemPoly.toString(), 8, problemPoly.getNumberOfVertices());
      FramePoint2D pointForTesting = new FramePoint2D();
      pointForTesting.set(-ankleToMidY, -ankleToMidX - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(0), epsilon);
      pointForTesting.set(-ankleToHeelX, +ankleToHeelY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(1), epsilon);
      pointForTesting.set(ankleToMidX, +ankleToMidY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(2), epsilon);
      pointForTesting.set(ankleToToeX, ankleToToeY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(3), epsilon);
      pointForTesting.set(ankleToToeX, -ankleToToeY + width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(4), epsilon);
      pointForTesting.set(ankleToMidY, -ankleToMidX - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(5), epsilon);
      pointForTesting.set(ankleToToeY, -ankleToToeX - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(6), epsilon);
      pointForTesting.set(-ankleToToeY, -ankleToToeX - width);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointForTesting, problemPoly.getVertex(7), epsilon);
   }

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
      ContactStatePlanGenerator generator = new ContactStatePlanGenerator(12, 1e-5);
      ContactState contactStateToTest = new ContactState();
      generator.computeAndSetSupportPolygon(contactStateToTest, new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, 0.0), 0.0),
                                            new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, width), 0.0),
                                            new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -width), 0.0), defaultFootPolygon,
                                            defaultFootPolygon);
      assertTrue(contactStateToTest.getReferenceFrame() == ReferenceFrame.getWorldFrame());
      assertTrue(contactStateToTest.isSupported());
      ConvexPolygon2D polygonToTest = new ConvexPolygon2D();
      contactStateToTest.getSupportPolygon(polygonToTest);
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
      ContactStatePlanGenerator generator = new ContactStatePlanGenerator(12, 1e-5);
      ContactState contactStateToTest = new ContactState();
      generator.computeAndSetSupportPolygon(contactStateToTest, new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, 0.0), Math.PI / 2),
                                            new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, width), 0.0),
                                            new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0, -width), 0.0), defaultFootPolygon,
                                            defaultFootPolygon);
      assertTrue(contactStateToTest.getReferenceFrame() == ReferenceFrame.getWorldFrame());
      assertTrue(contactStateToTest.isSupported());
      ConvexPolygon2D polygonToTest = new ConvexPolygon2D();
      contactStateToTest.getSupportPolygon(polygonToTest);
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
      ContactStatePlanGenerator contactStatePlanGenerationHelper = new ContactStatePlanGenerator(12, 1e-5);
      contactStatePlanGenerationHelper.generateContactStatePlanForJumping(contactStates, numberOfJumps, initialPelvisPose, pelvisPoseChangePerJump,
                                                                          leftAnklePoseOffset, rightAnklePoseOffset, 0.1, 0.3, defaultSupportPolygonAnkleFrame);
      Stream.of(contactStates).forEach((state) -> PrintTools.debug(state.toString()));
   }

   @Test(timeout = 200)
   public void testContactStateSupportPolygonSetting()
   {
      ContactState contactStateToTest = new ContactState(ReferenceFrame.getWorldFrame());
      contactStateToTest.setSupportPolygon(generateSimpleFootSupportPolygon());
      contactStateToTest.setPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 0.0));
      contactStateToTest.setOrientation(new FrameQuaternion(ReferenceFrame.getWorldFrame(), Math.PI / 2, 0.0, 0.0));
      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d();
      contactStateToTest.getSupportPolygon(ReferenceFrame.getWorldFrame(), polygon);
   }
}