package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

public class ChebyshevNodeGenerator
{
   /**
    * Populates the list provided with roots of the nth Chebyshev polynomial of the first kind
    * @param listToPopulate the list in which computed values are stored. The list is not cleared and entries are simply appended using the {@code List#add()} method
    * @param n the order of the Chebyshev polynomial or number of nodes to generate
    */
   public static void generateChebyshevNodes(List<Double> listToPopulate, int n)
   {
      for (int i = 0; i < n; i++)
         listToPopulate.add(Math.cos((2.0 * i + 1.0) / (2.0 * n) * Math.PI));
   }

   /**
    * Populates the list by mapping roots of the nth Chebyshev polynomial of the first kind to the interval [x0, xf]
    * @param listToPopulate the list in which computed values are stored. The list is not cleared and entries are simply appended using the {@code List#add()} method
    * @param n the order of the Chebyshev polynomial or number of nodes to generate
    */
   public static void generateChebyshevNodesWithArbitraryRange(List<Double> listToPopulate, double x0, double xF, int n)
   {
      for (int i = 0; i < n; i++)
         listToPopulate.add(((Math.cos((2.0 * i + 1.0) / (2.0 * n) * Math.PI)) * (xF - x0) + xF + x0) * 0.5);
   }

   /**
    * 
    * @param listToPoulate
    * @param x0
    * @param xF
    * @param n
    */
   public static void generateChebyshevNodesAndIncludeEndPoints(List<Double> listToPoulate, double x0, double xF, int n)
   {
      
   }
}
