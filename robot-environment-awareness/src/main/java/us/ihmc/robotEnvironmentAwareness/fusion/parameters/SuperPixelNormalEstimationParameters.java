package us.ihmc.robotEnvironmentAwareness.fusion.parameters;

import us.ihmc.jOctoMap.tools.ScannerTools;

import java.util.Scanner;

public class SuperPixelNormalEstimationParameters
{
   public static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.02;
   public static final double DEFAULT_MIN_CONSENSUS_RATIO = 0.5;
   public static final double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO = 0.75;
   public static final int DEFAULT_NUMBER_OF_ITERATIONS = 10;
   public static final boolean DEFAULT_LEAST_SQUARES_ESTIMATION = true;
   public static final boolean DEFAULT_UPDATE_USING_PCA = true;
   public static final int DEFAULT_MAX_ATTEMPTS_FOR_NORMAL_GUESS = 10;
   public static final double DEFAULT_MIN_DISTANCE_FOR_NORMAL_GUESS = 0.01;

   private double maxDistanceFromPlane;

   private double minConsensusRatio;
   private double maxAverageDeviationRatio;
   private double minDistanceForNormalGuess;

   private int numberOfIterations;
   private boolean enableLeastSquaresEstimation;

   private int maxAttemptsForNormalGuess;
   private boolean updateUsingPCA;

   public SuperPixelNormalEstimationParameters()
   {
      setDefaultParameters();
   }

   public SuperPixelNormalEstimationParameters(SuperPixelNormalEstimationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      maxDistanceFromPlane = DEFAULT_MAX_DISTANCE_FROM_PLANE;
      minConsensusRatio = DEFAULT_MIN_CONSENSUS_RATIO;
      maxAverageDeviationRatio = DEFAULT_MAX_AVERAGE_DEVIATION_RATIO;
      numberOfIterations = DEFAULT_NUMBER_OF_ITERATIONS;
      enableLeastSquaresEstimation = DEFAULT_LEAST_SQUARES_ESTIMATION;
      updateUsingPCA = DEFAULT_UPDATE_USING_PCA;
      maxAttemptsForNormalGuess = DEFAULT_MAX_ATTEMPTS_FOR_NORMAL_GUESS;
      minDistanceForNormalGuess = DEFAULT_MIN_DISTANCE_FOR_NORMAL_GUESS;
   }

   public void set(SuperPixelNormalEstimationParameters other)
   {
      maxDistanceFromPlane = other.maxDistanceFromPlane;
      minConsensusRatio = other.minConsensusRatio;
      maxAverageDeviationRatio = other.maxAverageDeviationRatio;
      numberOfIterations = other.numberOfIterations;
      maxAttemptsForNormalGuess = other.maxAttemptsForNormalGuess;
      enableLeastSquaresEstimation = other.enableLeastSquaresEstimation;
      updateUsingPCA = other.updateUsingPCA;
      minDistanceForNormalGuess = other.minDistanceForNormalGuess;
   }

   public void setMaxDistanceFromPlane(double maxDistanceFromPlane)
   {
      this.maxDistanceFromPlane = maxDistanceFromPlane;
   }

   public void setMinConsensusRatio(double minConsensusRatio)
   {
      this.minConsensusRatio = minConsensusRatio;
   }

   public void setMaxAverageDeviationRatio(double maxAverageDeviationRatio)
   {
      this.maxAverageDeviationRatio = maxAverageDeviationRatio;
   }

   public void setNumberOfIterations(int numberOfIterations)
   {
      this.numberOfIterations = numberOfIterations;
   }

   public void enableLeastSquaresEstimation(boolean enableLeastSquaresEstimation)
   {
      this.enableLeastSquaresEstimation = enableLeastSquaresEstimation;
   }

   public int getMaxAttemptsForNormalGuess()
   {
      return maxAttemptsForNormalGuess;
   }

   public double getMinDistanceForNormalGuess()
   {
      return minDistanceForNormalGuess;
   }

   public void updateUsingPCA(boolean updateUsingPCA)
   {
      this.updateUsingPCA = updateUsingPCA;
   }


   public double getMaxDistanceFromPlane()
   {
      return maxDistanceFromPlane;
   }

   public double getMinConsensusRatio()
   {
      return minConsensusRatio;
   }

   public double getMaxAverageDeviationRatio()
   {
      return maxAverageDeviationRatio;
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public boolean isLeastSquaresEstimationEnabled()
   {
      return enableLeastSquaresEstimation;
   }

   public void setMaxAttemptsForNormalGuess(int maxAttemptsForNormalGuess)
   {
      this.maxAttemptsForNormalGuess = maxAttemptsForNormalGuess;
   }

   public void setMinDistanceForNormalGuess(double minDistanceForNormalGuess)
   {
      this.minDistanceForNormalGuess = minDistanceForNormalGuess;
   }

   public boolean updateUsingPCA()
   {
      return updateUsingPCA;
   }

   @Override
   public String toString()
   {
      return "max distance from plane: " + maxDistanceFromPlane
            + ", min consensus ratio: " + minConsensusRatio + ", max average deviation ratio: " + maxAverageDeviationRatio
            + ", number of iterations: " + numberOfIterations + ", least squares estimation: " + enableLeastSquaresEstimation
            + ", update using pca: " + updateUsingPCA + ", max attempts for normal guess: " + maxAttemptsForNormalGuess
            + ", min distance for normal guess: " + minDistanceForNormalGuess;
   }

   public static SuperPixelNormalEstimationParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      SuperPixelNormalEstimationParameters parameters = new SuperPixelNormalEstimationParameters();
      parameters.setMaxDistanceFromPlane(ScannerTools.readNextDouble(scanner, parameters.getMaxDistanceFromPlane()));
      parameters.setMinConsensusRatio(ScannerTools.readNextDouble(scanner, parameters.getMinConsensusRatio()));
      parameters.setMaxAverageDeviationRatio(ScannerTools.readNextDouble(scanner, parameters.getMaxAverageDeviationRatio()));
      parameters.setNumberOfIterations(ScannerTools.readNextInt(scanner, parameters.getNumberOfIterations()));
      parameters.enableLeastSquaresEstimation(ScannerTools.readNextBoolean(scanner, parameters.isLeastSquaresEstimationEnabled()));
      parameters.updateUsingPCA(ScannerTools.readNextBoolean(scanner, parameters.updateUsingPCA()));
      parameters.setMinDistanceForNormalGuess(ScannerTools.readNextDouble(scanner, parameters.getMinDistanceForNormalGuess()));
      parameters.setMaxAttemptsForNormalGuess(ScannerTools.readNextInt(scanner, parameters.getMaxAttemptsForNormalGuess()));
      scanner.close();
      return parameters;
   }
}
