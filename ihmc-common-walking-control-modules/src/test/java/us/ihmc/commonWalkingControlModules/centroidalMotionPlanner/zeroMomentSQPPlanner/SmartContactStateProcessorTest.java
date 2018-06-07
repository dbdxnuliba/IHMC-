package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;

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
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SmartContactStateProcessorTest
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

   private ConvexPolygon2D generateDefaultFootSupportPolygon()
   {
      return new ConvexPolygon2D(generateDefaultFootSupportPolygonVertexList());
   }

   @Test(timeout = 200)
   public void testMinimalVertexPolygonGeneration()
   {
      double epsilon = Epsilons.ONE_BILLIONTH;
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
      SmartContactStateProcessor.generateMinimalVertexSupportPolygon(problemPoly, vertexList, defaultFootPolygonPointsInAnkleFrame.size() * 2, 1e-5);
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
      SmartContactStateProcessor.generateMinimalVertexSupportPolygon(problemPoly, vertexList, defaultFootPolygonPointsInAnkleFrame.size() * 2, 1e-5);
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

   @Test(timeout = 1000)
   public void testNodeGeneration()
   {
      List<Double> nodeList = new ArrayList<>();
      SmartContactStateProcessor.generateChebyshevNodes(nodeList, 4);
      assertEquals(4, nodeList.size());
      for (int i = 0; i < 4; i++)
         PrintTools.debug(nodeList.get(i) + " ");
   }

   @Test(timeout = 1000)
   public void testNodeGenerationArbitraryRange()
   {
      int n = 6;
      List<Double> nodeList = new ArrayList<>();
      SmartContactStateProcessor.generateChebyshevNodesWithArbitraryRange(nodeList, 0.1, 0.8, n);
      assertEquals(n, nodeList.size());
      String str = "";
      for (int i = 0; i < nodeList.size(); i++)
         str += Double.toString(nodeList.get(i)) + "\n";
      PrintTools.debug(str);
   }

   @Test(timeout = 1000)
   public void testBeginningIntervalRange()
   {
      List<Double> nodeList = new ArrayList<>();
      SmartContactStateProcessor.appendChebyshevNodesBiasedTowardsIntervalBeginning(nodeList, 0.1, 0.45, 3);
      for (int i = 0; i < nodeList.size(); i++)
         PrintTools.debug(i + ": " + nodeList.get(i));
   }

   @Test(timeout = 1000)
   public void testEndIntervalRange()
   {
      List<Double> nodeList = new ArrayList<>();
      SmartContactStateProcessor.appendChebyshevNodesBiasedTowardsIntervalEnd(nodeList, 0.45, 0.8, 3);
      for (int i = 0; i < nodeList.size(); i++)
         PrintTools.debug(i + ": " + nodeList.get(i));
   }

   @Test(timeout = 1000)
   public void testContactStateProcessing()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      SmartContactStateProcessor contactStateProcessor = new SmartContactStateProcessor("Test", registry);
      contactStateProcessor.initialize(2, 12);
      List<ContactState> contactStateList = new ArrayList<>();
      ConvexPolygon2D supportPolygon = generateDefaultFootSupportPolygon();
      ContactState groundState = new ContactState();
      groundState.setDuration(0.4);
      ContactState flightState = new ContactState();
      flightState.setDuration(0.1);
      for (RobotSide side : RobotSide.values)
      {
         groundState.setSupportPolygon(side, supportPolygon);
         groundState.setFootInContact(side, true);
         groundState.setFootPose(side, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(0.0, side.negateIfRightSide(0.15), 0.0), new Quaternion()));
         flightState.setFootInContact(side, false);
      }
      contactStateList.add(groundState);
      contactStateList.add(flightState);
      contactStateList.add(groundState);
      RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentList = new RecyclingArrayList<>(CollinearForceMotionPlannerSegment.class);
      contactStateProcessor.processContactStates(contactStateList, segmentList);
      assertEquals(7, segmentList.size());
      double cummulativeTime = 0.0;
      for (int i = 0; i < segmentList.size(); i++)
      {
         cummulativeTime += segmentList.get(i).getSegmentDuration();
         PrintTools.debug("Segment " + i + ": " + segmentList.get(i).getSegmentDuration() + ", Cummulative: " + cummulativeTime);
      }
   }
}
