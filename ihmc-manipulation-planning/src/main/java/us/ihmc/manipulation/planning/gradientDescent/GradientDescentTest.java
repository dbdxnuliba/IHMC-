package us.ihmc.manipulation.planning.gradientDescent;

import gnu.trove.list.array.TDoubleArrayList;

public class GradientDescentTest
{
   public GradientDescentTest()
   {
      System.out.println("Hello Test");

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.add(35.0);
      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            return Math.pow((values.get(0) - 10) * 10, 2.0) + 5;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);

      TDoubleArrayList upperLimit = new TDoubleArrayList();
      upperLimit.add(35.0);
      solver.setInputUpperLimit(upperLimit);

      System.out.println("iteration is " + solver.run());
      TDoubleArrayList optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      System.out.println("optimal query is " + solver.getOptimalQuery());

      System.out.println("Good Bye Test");
   }

   public static void main(String[] args)
   {
      new GradientDescentTest();
   }
}
