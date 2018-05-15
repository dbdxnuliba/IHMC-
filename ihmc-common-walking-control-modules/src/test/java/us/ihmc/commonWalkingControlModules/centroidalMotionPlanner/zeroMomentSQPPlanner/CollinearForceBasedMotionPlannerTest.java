package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.flight.BipedContactType;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
      tempContactState.setContactType(BipedContactType.DOUBLE_SUPPORT);
      tempContactState.setDuration(0.4);
      tempContactState.setPose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setContactType(BipedContactType.NO_SUPPORT);
      tempContactState.setDuration(0.2);
      tempContactState.setPose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setContactType(BipedContactType.DOUBLE_SUPPORT);
      tempContactState.setDuration(0.4);
      tempContactState.setPose(tempPose);
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
      tempContactState.setContactType(BipedContactType.DOUBLE_SUPPORT);
      tempContactState.setDuration(0.411);
      tempContactState.setPose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setContactType(BipedContactType.NO_SUPPORT);
      tempContactState.setDuration(0.04);
      tempContactState.setPose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.0, 0.0, 0.1, 0.04);
      motionPlanner.appendContactStateToList(tempContactState);

      tempContactState.reset();
      tempContactState.setContactType(BipedContactType.DOUBLE_SUPPORT);
      tempContactState.setDuration(0.4);
      tempContactState.setPose(tempPose);
      generateSupportPolygon(tempSupportPolygon, 0.1, 0.0, 0.1, 0.04);
      tempContactState.setSupportPolygon(tempSupportPolygon);
      motionPlanner.appendContactStateToList(tempContactState);

      motionPlanner.runIterations(0);
      List<CollinearForceMotionPlannerSegment> segmentList = motionPlanner.getSegmentList();
      assertEquals(18, segmentList.size());
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
}
