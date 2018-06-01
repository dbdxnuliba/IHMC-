package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SmartContactStateProcessorTest
{
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
      contactStateProcessor.initialize(2);
      List<ContactState> contactStateList = new ArrayList<>();
      ContactState groundState = new ContactState();
      groundState.setDuration(0.4);
      ConvexPolygon2D supportPolygon = new ConvexPolygon2D(Stream.of(new Point2D(0.1, 0.1), new Point2D(0.1, -0.1), new Point2D(-0.1, -0.1),
                                                                     new Point2D(-0.1, 0.1))
                                                                 .collect(Collectors.toList()));
      groundState.setSupportPolygon(supportPolygon);
      ContactState flightState = new ContactState();
      flightState.setDuration(0.1);
      supportPolygon.clearAndUpdate();
      flightState.setSupportPolygon(supportPolygon);
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
