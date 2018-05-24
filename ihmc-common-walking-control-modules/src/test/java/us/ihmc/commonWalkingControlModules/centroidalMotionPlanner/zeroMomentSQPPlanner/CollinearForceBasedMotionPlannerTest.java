package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollinearForceBasedMotionPlannerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private YoVariableRegistry registry;
   private CollinearForceBasedCoMMotionPlanner motionPlanner;
   private static final FrameVector3D gravity = new FrameVector3D(worldFrame, 0.0, 0.0, -9.81);

   @Before
   public void setupTest()
   {
      registry = new YoVariableRegistry("MotionPlannerTestRegistry");
      motionPlanner = new CollinearForceBasedCoMMotionPlanner(gravity, registry);
      motionPlanner.initialize(new CollinearForcePlannerParameters());
   }

   @Test
   public void testContactStateProcessing()
   {
      ContactState tempContactState = new ContactState();
      ConvexPolygon2D tempSupportPolygon = new ConvexPolygon2D();
      FramePose3D tempPose = new FramePose3D(worldFrame);

      tempContactState.reset();
      tempContactState.setDuration(0.4);
      tempContactState.setSupportPolygonFramePose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setDuration(0.2);
      tempContactState.setSupportPolygonFramePose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setDuration(0.4);
      tempContactState.setSupportPolygonFramePose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.1, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      List<ContactState> contactStateList = motionPlanner.getContactStateList();
      //for (int i = 0; i < contactStateList.size(); i++)
      //   PrintTools.debug(contactStateList.get(i).toString());
      assertTrue(contactStateList.size() == 3);
   }

   @Test
   public void testContactStateToSegmentTranslation()
   {
      ContactState tempContactState = new ContactState();
      ConvexPolygon2D tempSupportPolygon = new ConvexPolygon2D();
      FramePose3D tempPose = new FramePose3D(worldFrame);

      tempContactState.reset();
      tempContactState.setDuration(0.40);
      tempContactState.setSupportPolygonFramePose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setDuration(0.1);
      tempContactState.setSupportPolygonFramePose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setDuration(0.4);
      tempContactState.setSupportPolygonFramePose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.1, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      motionPlanner.runIterations(0);
      List<CollinearForceMotionPlannerSegment> segmentList = motionPlanner.getSegmentList();
      //assertEquals(18, segmentList.size());
      for(int i = 0; i < segmentList.size(); i++)
      {
         PrintTools.debug(segmentList.get(i).toString());
      }
   }
   
   @Test 
   public void testInitialGuessGeneration()
   {
      assertTrue(false);
   }

   @Test
   public void testIterations()
   {
      assertTrue(false);
   }
   
   @Test
   public void testConvergence()
   {
      assertTrue(false);
   }

   private void generateSupportPolygon(ConvexPolygon2D polygonToSet, double centroidX, double centroidY, double length, double width)
   {
      polygonToSet.clear();
      for (int i = 0; i < 4; i++)
         polygonToSet.addVertex(centroidX + Math.pow(-1.0, i) * length / 2.0, centroidY + Math.pow(-1.0, i / 2) * width / 2.0);
      polygonToSet.update();
   }

   @Test
   public void testPolygonJoining()
   {
      ConvexPolygon2D problemPoly = new ConvexPolygon2D();
      double width = 0.1;
      double ankleToToeX = 0.0725;
      double ankleToToeY = 0.0225;
      double ankleToMidX = 0.03625;
      double ankleToMidY = 0.04;
      double ankleToHeelX = 0.0175;
      double ankleToHeelY = 0.02;
      List<Point2D> defaultFootPolygonPointsInAnkleFrame = Stream.of(new Point2D(ankleToToeX, ankleToToeY), new Point2D(ankleToToeX, -ankleToToeY),
                                                                                   new Point2D(ankleToMidX, -ankleToMidY),
                                                                                   new Point2D(-ankleToHeelX, -ankleToHeelY),
                                                                                   new Point2D(-ankleToHeelX, ankleToHeelY), new Point2D(ankleToMidX, ankleToMidY))
                                                                               .collect(Collectors.toList());
      problemPoly.clear();
      for(int i = 0; i < defaultFootPolygonPointsInAnkleFrame.size(); i++)
      {
         problemPoly.addVertex(defaultFootPolygonPointsInAnkleFrame.get(i).getX(), defaultFootPolygonPointsInAnkleFrame.get(i).getY() + width);
         problemPoly.addVertex(defaultFootPolygonPointsInAnkleFrame.get(i).getX(), defaultFootPolygonPointsInAnkleFrame.get(i).getY() - width);
      }
      problemPoly.update();
      assertEquals("Resultant poly is " + problemPoly.toString(), 6, problemPoly.getNumberOfVertices());
   }
}
