package us.ihmc.manipulation.planning.gradientDescent;

public class GradientDescentTest
{
   public GradientDescentTest()
   {
      System.out.println("Hello Test");

      double initialInput = 35.0;
      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(double... values)
         {
            return Math.pow((values[0] - 10) * 10, 2.0) + 5;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initialInput);

      solver.setInputUpperLimit(35.0);

      System.out.println("iteration is " + solver.run());
      double[] optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.length; i++)
         System.out.println("solution is " + optimalSolution[i]);

      System.out.println("optimal query is " + solver.getOptimalQuery());

      System.out.println("Good Bye Test");
   }

   public static void main(String[] args)
   {
      new GradientDescentTest();
   }
}
