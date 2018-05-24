package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
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
      FrameConvexPolygon2d problemPoly = new FrameConvexPolygon2d();
      List<Point2D> defaultFootPolygonPointsInAnkleFrame = Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                     new Point2D(ankleToMidX, -ankleToMidY), new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                     new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                 .collect(Collectors.toList());
      ArrayList<FramePoint2D> vertexList = new ArrayList<>();
      for (int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         FramePoint2D leftFootPoint = new FramePoint2D();
         leftFootPoint.set(ReferenceFrame.getWorldFrame(), defaultFootPolygonPointsInAnkleFrame.get(i).getX(),
                           defaultFootPolygonPointsInAnkleFrame.get(i).getY() + width);
         FramePoint2D rightFootPoint = new FramePoint2D();
         rightFootPoint.set(ReferenceFrame.getWorldFrame(), defaultFootPolygonPointsInAnkleFrame.get(i).getX(),
                            defaultFootPolygonPointsInAnkleFrame.get(i).getY() - width);
         vertexList.add(leftFootPoint);
         vertexList.add(rightFootPoint);
      }
      vertexList.add(new FramePoint2D());
      vertexList.add(new FramePoint2D());
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
      FrameConvexPolygon2d problemPoly = new FrameConvexPolygon2d();
      List<Point2D> defaultFootPolygonPointsInAnkleFrame = Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                     new Point2D(ankleToMidX, -ankleToMidY), new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                     new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                 .collect(Collectors.toList());
      ArrayList<FramePoint2D> vertexList = new ArrayList<>();
      for (int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         FramePoint2D leftFootPoint = new FramePoint2D();
         leftFootPoint.set(ReferenceFrame.getWorldFrame(), defaultFootPolygonPointsInAnkleFrame.get(i).getX(),
                           defaultFootPolygonPointsInAnkleFrame.get(i).getY() + width);
         FramePoint2D rightFootPoint = new FramePoint2D();
         rightFootPoint.set(ReferenceFrame.getWorldFrame(), defaultFootPolygonPointsInAnkleFrame.get(i).getY(),
                            -defaultFootPolygonPointsInAnkleFrame.get(i).getX() - width);
         vertexList.add(leftFootPoint);
         vertexList.add(rightFootPoint);
      }
      vertexList.add(new FramePoint2D());
      vertexList.add(new FramePoint2D());
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

   @Test(timeout = 200)
   public void testContactStateSupportPolygonGeneration()
   {
      double epsilon = Epsilons.ONE_BILLIONTH;
      ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D(Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                         new Point2D(ankleToMidX, -ankleToMidY), new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                         new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                     .collect(Collectors.toList()));
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
      ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D(Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                         new Point2D(ankleToMidX, -ankleToMidY), new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                         new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                     .collect(Collectors.toList()));
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
}