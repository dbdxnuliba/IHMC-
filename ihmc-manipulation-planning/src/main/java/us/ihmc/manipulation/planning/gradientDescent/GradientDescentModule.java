package us.ihmc.manipulation.planning.gradientDescent;

import us.ihmc.commons.MathTools;

public class GradientDescentModule
{
   // internal
   private SingleQueryFunction function;
   private final int dimension;
   private final double[] initialInput;

   // result
   private boolean solved;
   private double[] optimalInput;
   private double optimalQuery;

   // params
   private double[] inputUpperLimit;
   private double[] inputLowerLimit;
   private double deltaThreshold = 10E-10;
   private int maximumIterations = 1000;
   private double alpha = -1;
   private double perturb = 0.001;

   public GradientDescentModule(SingleQueryFunction function, double... initialInput)
   {
      this.function = function;
      this.dimension = initialInput.length;
      this.initialInput = new double[dimension];
      this.optimalInput = new double[dimension];
      this.inputUpperLimit = new double[dimension];
      this.inputLowerLimit = new double[dimension];
      for (int i = 0; i < dimension; i++)
      {
         this.initialInput[i] = initialInput[i];
         this.optimalInput[i] = 0.0;
         this.inputUpperLimit[i] = Double.MAX_VALUE;
         this.inputLowerLimit[i] = Double.MIN_VALUE;
      }
   }

   private void reduceStepSize()
   {
      alpha = alpha * 0.1;
   }

   public void setMaximumIterations(int value)
   {
      maximumIterations = value;
   }

   public void setInputUpperLimit(double... limit)
   {
      for (int i = 0; i < dimension; i++)
         inputUpperLimit[i] = limit[i];
   }

   public void setInputLowerLimit(double... limit)
   {
      for (int i = 0; i < dimension; i++)
         inputLowerLimit[i] = limit[i];
   }

   public int run()
   {
      solved = false;
      optimalQuery = Double.MAX_VALUE;

      int iteration = 0;
      double[] pastInput = new double[dimension];
      for (int i = 0; i < dimension; i++)
         pastInput[i] = initialInput[i];
      for (int i = 0; i < maximumIterations; i++)
      {
         iteration++;
         double pastQuery = function.getQuery(pastInput);

         double tempSignForPerturb = 1.0;

         double[] gradient = new double[dimension];
         for (int j = 0; j < dimension; j++)
         {
            double[] perturbedInput = new double[dimension];
            for (int k = 0; k < dimension; k++)
               perturbedInput[k] = pastInput[k];

            if (perturbedInput[j] == inputUpperLimit[j])
               tempSignForPerturb = -1.0;

            if (perturbedInput[j] == inputUpperLimit[j])
               ;//System.out.println("current input is meeting with upper limit");

            double tempInput = perturbedInput[j] + perturb * tempSignForPerturb;

            perturbedInput[j] = MathTools.clamp(tempInput, inputLowerLimit[j], inputUpperLimit[j]);

            double perturbedQuery = function.getQuery(perturbedInput);

            gradient[j] = (perturbedQuery - pastQuery) / (perturb * tempSignForPerturb);
         }

         for (int j = 0; j < dimension; j++)
         {
            double input = pastInput[j] + gradient[j] * alpha;
            optimalInput[j] = MathTools.clamp(input, inputLowerLimit[j], inputUpperLimit[j]);
         }

         optimalQuery = function.getQuery(optimalInput);

         if (optimalQuery > pastQuery)
         {
            reduceStepSize();
            for (int j = 0; j < dimension; j++)
               optimalInput[j] = pastInput[j];
         }

         double delta = Math.abs((pastQuery - optimalQuery) / optimalQuery);

         //System.out.println(i + " " + optimalQuery + " " + optimalInput[0] + " " + gradient[0] + " " + delta);

         if (delta < deltaThreshold)
            break;

         for (int j = 0; j < dimension; j++)
            pastInput[j] = optimalInput[j];
      }

      return iteration;
   }

   public boolean isSolved()
   {
      return solved;
   }

   public double[] getOptimalInput()
   {
      return optimalInput;
   }

   public double getOptimalQuery()
   {
      return optimalQuery;
   }
}
