package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.PrintTools;

public class ChebyshevNodeGeneratorTest
{
   @Test(timeout = 1000)
   public void testNodeGeneration()
   {
      List<Double> nodeList = new ArrayList<>();
      ChebyshevNodeGenerator.generateChebyshevNodes(nodeList, 4);
      assertEquals(4, nodeList.size());
      for(int i = 0; i < 4; i++)
         PrintTools.debug(nodeList.get(i) + " ");
   }

   @Test(timeout = 1000)
   public void testNodeGenerationArbitraryRange()
   {
      List<Double> nodeList = new ArrayList<>();
      ChebyshevNodeGenerator.generateChebyshevNodesWithArbitraryRange(nodeList, 0.0, 0.4, 2);
      assertEquals(2, nodeList.size());
      String str = "";
      for(int i = 0; i < nodeList.size(); i++)
         str += Double.toString(nodeList.get(i)) + "\n";
      PrintTools.debug(str);
   }
}
