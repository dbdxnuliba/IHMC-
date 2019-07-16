package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.Assert;
import org.junit.jupiter.api.Test;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.capturePoint.numerical.SupportSeqence;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SupportSequenceTest
{
   @Test
   public void testSupportSequence()
   {
      MutableDouble time = new MutableDouble();
      ConvexPolygon2D defaultSupportPolygon = createDefaultSupportPolygon();
      SideDependentList<PoseReferenceFrame> soleFrames = createSoleFrames();
      SupportSeqence supportSeqence = new SupportSeqence(defaultSupportPolygon, soleFrames, () -> time.getValue());

      Footstep footstep = new Footstep(RobotSide.LEFT);
      footstep.setX(0.2);
      footstep.setY(0.1);
      FootstepTiming timing = new FootstepTiming(0.5, 0.2);
      timing.setLiftoffDuration(0.1);

      List<Footstep> footsteps = Arrays.asList(footstep);
      List<FootstepTiming> timings = Arrays.asList(timing);
      supportSeqence.setFromFootsteps(footsteps, timings);

      List<ConvexPolygon2D> expectedSupportPolygons = new ArrayList<>();
      TDoubleList expectedSupportTimes = new TDoubleArrayList();
      // At time 0.0 we expect full support
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, 0.15);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, 0.15);
         polygon.addVertex(-0.1, -0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.0);
      }
      // At time 0.1 the robot should toe off with the left foot
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, 0.15);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(-0.1, -0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.1);
      }
      // At time 0.2 only the right foot should remain in contact
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, -0.05);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(-0.1, -0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.2);
      }
      // At time 0.7 the swing foot will have touched down
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.3, 0.15);
         polygon.addVertex(0.3, 0.05);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(0.1, 0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.7);
      }

      List<? extends ConvexPolygon2DReadOnly> supportPolygons = supportSeqence.getSupportPolygons();
      TDoubleList supportTimes = supportSeqence.getSupportTimes();
      for (int i = 0; i < supportPolygons.size(); i++)
      {
         EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(expectedSupportPolygons.get(i), supportPolygons.get(i), 1.0e-1);
         Assert.assertEquals(expectedSupportTimes.get(i), supportTimes.get(i), 1.0e-10);
      }
   }

   private static SideDependentList<PoseReferenceFrame> createSoleFrames()
   {
      SideDependentList<PoseReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getLowerCaseName() + "SoleFrame", ReferenceFrame.getWorldFrame());
         soleFrame.setPositionWithoutChecksAndUpdate(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         soleFrames.put(robotSide, soleFrame);
      }
      return soleFrames;
   }

   private static ConvexPolygon2D createDefaultSupportPolygon()
   {
      ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
      defaultSupportPolygon.addVertex(0.1, 0.05);
      defaultSupportPolygon.addVertex(0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, 0.05);
      defaultSupportPolygon.addVertex(-0.1, -0.05);
      defaultSupportPolygon.update();
      return defaultSupportPolygon;
   }
}
