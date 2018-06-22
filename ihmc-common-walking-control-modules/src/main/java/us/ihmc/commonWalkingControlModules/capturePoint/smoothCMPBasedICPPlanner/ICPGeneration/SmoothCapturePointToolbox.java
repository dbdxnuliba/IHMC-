package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CNormOps;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

/**
 * @author Tim Seyde
 */

public class SmoothCapturePointToolbox
{
   private static final int defaultSize = 100;
   private final DenseMatrix64F omegaPowers;
   private final DenseMatrix64F derivativeCoefficients;
   private final DenseMatrix64F initialTimeDerivativePowers;
   private final DenseMatrix64F finalTimeDerivativePowers;
   private final DenseMatrix64F currentTimeDerivativePowers;
   private final DenseMatrix64F tempCMPDerivatives;

   private final DenseMatrix64F generalizedAlphaPrimeRow;
   private final DenseMatrix64F generalizedBetaPrimeRow;

   private final DenseMatrix64F polynomialCoefficientCombinedVector;
   private final DenseMatrix64F polynomialCoefficientVector;

   private final DenseMatrix64F generalizedAlphaPrimeMatrix;
   private final DenseMatrix64F generalizedBetaPrimeMatrix;
   private final DenseMatrix64F generalizedAlphaBetaPrimeMatrix;

   private final DenseMatrix64F M1;
   private final int maxNumberOfCMPCoefficents;

   public SmoothCapturePointToolbox()
   {
      this(defaultSize);
   }

   public SmoothCapturePointToolbox(int maxNumberCMPCoefficients)
   {
      this.maxNumberOfCMPCoefficents = maxNumberCMPCoefficients;
      omegaPowers = new DenseMatrix64F(1, maxNumberCMPCoefficients);
      derivativeCoefficients = generateDerivativeCoefficientMatrix(maxNumberCMPCoefficients);
      initialTimeDerivativePowers = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
      currentTimeDerivativePowers = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
      finalTimeDerivativePowers = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
      tempCMPDerivatives = new DenseMatrix64F(0, 1);

      generalizedAlphaPrimeRow = new DenseMatrix64F(1, maxNumberCMPCoefficients);
      generalizedBetaPrimeRow = new DenseMatrix64F(1, maxNumberCMPCoefficients);

      polynomialCoefficientCombinedVector = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
      polynomialCoefficientVector = new DenseMatrix64F(maxNumberCMPCoefficients, 1);

      generalizedAlphaPrimeMatrix = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
      generalizedBetaPrimeMatrix = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
      generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);

      M1 = new DenseMatrix64F(maxNumberCMPCoefficients, maxNumberCMPCoefficients);
   }

   public static DenseMatrix64F generateDerivativeCoefficientMatrix(int numberOfTrajectoryCoefficient)
   {
      DenseMatrix64F derivativeMatrix = new DenseMatrix64F(numberOfTrajectoryCoefficient, numberOfTrajectoryCoefficient);
      for (int i = 0; i < numberOfTrajectoryCoefficient; i++)
         derivativeMatrix.set(0, i, 1.0);
      for (int i = 1; i < numberOfTrajectoryCoefficient; i++)
      {
         for (int j = 0; j < i; j++)
            derivativeMatrix.set(i, j, 0.0);
         for (int j = i; j < numberOfTrajectoryCoefficient; j++)
            derivativeMatrix.set(i, j, derivativeMatrix.get(i - 1, j) * (j - i + 1));
      }
      return derivativeMatrix;
   }

   public static void calculateOmegaPowers(double omega0, DenseMatrix64F omega0PowersToSet, int numberOfCMPTrajectoryCoefficients)
   {
      omega0PowersToSet.reshape(1, numberOfCMPTrajectoryCoefficients);
      omega0PowersToSet.set(0, 0, 1.0);
      for (int i = 1; i < numberOfCMPTrajectoryCoefficients; i++)
         omega0PowersToSet.set(0, i, omega0PowersToSet.get(0, i - 1) / omega0);
   }

   public static void calculateTimePowers(DenseMatrix64F timePowersToSet, double time, int numberOfCoefficients, DenseMatrix64F derivativeCoefficients)
   {
      if (numberOfCoefficients > derivativeCoefficients.numCols || numberOfCoefficients > derivativeCoefficients.numCols)
         throw new RuntimeException("Insufficient derivative coefficients provided");

      timePowersToSet.reshape(numberOfCoefficients, numberOfCoefficients);
      double t_i = 1.0;
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         for (int j = 0; j + i < numberOfCoefficients; j++)
            timePowersToSet.set(j, j + i, derivativeCoefficients.get(j, j + i) * t_i);
         t_i *= time;
         for (int j = 0; j < i; j++)
            timePowersToSet.set(i, j, 0.0);
      }
   }

   public void calculateTimePowers(DenseMatrix64F timePowersToSet, double time, int numberOfCoefficientsInTrajectory)
   {
      if (numberOfCoefficientsInTrajectory > maxNumberOfCMPCoefficents)
         throw new RuntimeException("Toolbox was not initialized to handle the given trajectory size");
      calculateTimePowers(timePowersToSet, time, numberOfCoefficientsInTrajectory, derivativeCoefficients);
   }

   public static void computeCMPDerivatives(DenseMatrix64F cmpDerivativesToPack, DenseMatrix64F timeDerivativePowers, DenseMatrix64F cmpCoefficientVector,
                                            int orderOfHighestDerivative)
   {
      int numberOfDerivativesToSet = orderOfHighestDerivative + 1;
      if (numberOfDerivativesToSet > timeDerivativePowers.numRows || cmpCoefficientVector.numRows > timeDerivativePowers.numCols)
         throw new RuntimeException("Provided time derivative powers do not have adequate size to compute derivatives");
      cmpDerivativesToPack.reshape(numberOfDerivativesToSet, 1);
      for (int i = 0; i < numberOfDerivativesToSet; i++)
      {
         double derivative = 0.0;
         for (int j = 0; j < cmpCoefficientVector.numRows; j++)
            derivative += timeDerivativePowers.get(i, j) * cmpCoefficientVector.get(j, 0);
         cmpDerivativesToPack.set(i, 0, derivative);
      }
   }

   public static void calculateAlphaPrimeMatrix(DenseMatrix64F alphaPrimeMatrixToSet, int numberOfTrajectoryCoefficients, DenseMatrix64F timeDerivativePowers,
                                                DenseMatrix64F omegaPowers)
   {
      checkTimeDerivativePowerSize(numberOfTrajectoryCoefficients, timeDerivativePowers);
      checkOmegaPowerSize(numberOfTrajectoryCoefficients, omegaPowers);
      alphaPrimeMatrixToSet.reshape(1, numberOfTrajectoryCoefficients);
      for (int i = 0; i < numberOfTrajectoryCoefficients; i++)
      {
         double alphaPrimeMultipler = 0.0;
         for (int j = 0; j < numberOfTrajectoryCoefficients; j++)
         {
            double valueToAdd = omegaPowers.get(0, j) * timeDerivativePowers.get(j, i);
            alphaPrimeMultipler += valueToAdd;
         }
         alphaPrimeMatrixToSet.set(0, i, alphaPrimeMultipler);
      }
   }

   public static void calculateBetaPrimeMatrix(DenseMatrix64F betaPrimeMatrixToSet, int numberOfTrajectoryCoefficients,
                                               DenseMatrix64F finalTimeDerivativePowers, DenseMatrix64F omegaPowers, double gammaPrime)
   {
      checkTimeDerivativePowerSize(numberOfTrajectoryCoefficients, finalTimeDerivativePowers);
      checkOmegaPowerSize(numberOfTrajectoryCoefficients, omegaPowers);
      betaPrimeMatrixToSet.reshape(1, numberOfTrajectoryCoefficients);
      for (int i = 0; i < numberOfTrajectoryCoefficients; i++)
      {
         double betaPrimeMultiplier = 0.0;
         for (int j = 0; j < numberOfTrajectoryCoefficients; j++)
         {
            double valueToAdd = omegaPowers.get(0, j) * finalTimeDerivativePowers.get(j, i);
            betaPrimeMultiplier += valueToAdd;
         }
         betaPrimeMultiplier *= gammaPrime;
         betaPrimeMatrixToSet.set(0, i, betaPrimeMultiplier);
      }
   }

   private static void checkOmegaPowerSize(int numberOfTrajectoryCoefficients, DenseMatrix64F omegaPowers)
   {
      if (numberOfTrajectoryCoefficients > omegaPowers.numCols)
         throw new RuntimeException("Mismatch in number of trajectory coefficients (" + numberOfTrajectoryCoefficients + ") and omega powers matrix ("
               + omegaPowers.numCols + ")");
   }

   private static void checkTimeDerivativePowerSize(int numberOfTrajectoryCoefficients, DenseMatrix64F timeDerivativePowers)
   {
      if (numberOfTrajectoryCoefficients > timeDerivativePowers.numCols || numberOfTrajectoryCoefficients > timeDerivativePowers.numRows)
         throw new RuntimeException("Mismatch in number of trajectory coefficients (" + numberOfTrajectoryCoefficients + ") and time derivative powers matrix ("
               + timeDerivativePowers.numRows + ", " + timeDerivativePowers.numCols + ")");
   }

   private static void checkAlphaGammaPrimeMatrixSizeMatch(DenseMatrix64F alphaPrime, DenseMatrix64F betaPrime)
   {
      if (alphaPrime.numCols != betaPrime.numCols)
         throw new RuntimeException("Number of columns mismatch between alpha prime and beta prime matrix");
      if (alphaPrime.numRows != 1)
         throw new RuntimeException("Alpha prime has unexpected number of rows");
      if (betaPrime.numRows != 1)
         throw new RuntimeException("Alpha prime has unexpected number of rows");
   }

   private static void checkTrajectoryCoefficientsMatch(int numberOfCoefficients, DenseMatrix64F coefficients)
   {
      if (numberOfCoefficients != coefficients.numRows)
         throw new RuntimeException();
      if (coefficients.numCols != 1)
         throw new RuntimeException();
   }

   public static double calculateGammaPrimeMatrix(double omega0, double time, double finalTime)
   {
      return Math.exp(omega0 * (time - finalTime));
   }

   public static double calculateCapturePoint(DenseMatrix64F alphaPrimeMatrix, DenseMatrix64F betaPrimeMatrix, double gammaPrime,
                                              DenseMatrix64F cmpPolynomialCoefficients, double desiredFinalICPPosition)
   {
      checkAlphaGammaPrimeMatrixSizeMatch(alphaPrimeMatrix, betaPrimeMatrix);
      int numberOfCoefficients = alphaPrimeMatrix.getNumCols();
      checkTrajectoryCoefficientsMatch(numberOfCoefficients, cmpPolynomialCoefficients);
      double capturePoint = gammaPrime * desiredFinalICPPosition;
      for (int i = 0; i < numberOfCoefficients; i++)
         capturePoint += (alphaPrimeMatrix.get(0, i) - betaPrimeMatrix.get(0, i)) * cmpPolynomialCoefficients.get(i, 0);
      return capturePoint;
   }

   public void calculateICPQuantitiesFromCorrespondingCMPPolynomial1D(List<Double> icpQuantitiesToPopulate, double omega0, double time, int icpDerivativeOrder,
                                                                      Trajectory cmpPolynomial, double icpPositionDesiredFinal)
   {
      icpQuantitiesToPopulate.clear();
      double derivative = calculateICPPositionFromCorrespondingCMPPolynomial1D(omega0, time, cmpPolynomial, icpPositionDesiredFinal);
      computeCMPDerivatives(tempCMPDerivatives, currentTimeDerivativePowers, polynomialCoefficientVector, icpDerivativeOrder);
      icpQuantitiesToPopulate.add(derivative);
      for (int i = 0; i < icpDerivativeOrder; i++)
      {
         derivative = omega0 * (derivative - tempCMPDerivatives.get(i));
         icpQuantitiesToPopulate.add(derivative);
      }
   }

   public double calculateICPQuantityFromCorrespondingCMPPolynomial1D(double omega0, double time, int icpDerivativeOrder, Trajectory cmpPolynomial,
                                                                      double icpPositionDesiredFinal)
   {
      double derivative = calculateICPPositionFromCorrespondingCMPPolynomial1D(omega0, time, cmpPolynomial, icpPositionDesiredFinal);
      computeCMPDerivatives(tempCMPDerivatives, currentTimeDerivativePowers, polynomialCoefficientVector, icpDerivativeOrder);
      for (int i = 0; i < icpDerivativeOrder; i++)
         derivative = omega0 * (derivative - tempCMPDerivatives.get(i));
      return derivative;
   }

//   public double calculateICPQuantityFromCorrespondingCMPPolynomial1D(int icpDerivativeOrder, DenseMatrix64F omegaPowers, DenseMatrix64F alphaPrimeMatrix, DenseMatrix64F betaPrimeMatrix,
//                                                                      double gammaPrime, DenseMatrix64F cmpDerivatives, double desiredFinalICPPosition)
//   {
//      double derivative = calculateCapturePoint(alphaPrimeMatrix, betaPrimeMatrix, gammaPrime, cmpPolynomialCoefficients, desiredFinalICPPosition);
//      for (int i = 0; i < icpDerivativeOrder; i++)
//         derivative = omega0 * (derivative - tempCMPDerivatives.get(i));
//      return derivative;
//      
//   }

   public double calculateICPPositionFromCorrespondingCMPPolynomial1D(double omega0, double time, Trajectory cmpPolynomial, double icpPositionDesiredFinal)
   {
      double finalTime = cmpPolynomial.getFinalTime();
      double gammaPrime = calculateGammaPrimeMatrix(omega0, time, finalTime);
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      calculateTimePowers(currentTimeDerivativePowers, time, numberOfCoefficients);
      calculateTimePowers(finalTimeDerivativePowers, finalTime, numberOfCoefficients);
      calculateOmegaPowers(omega0, omegaPowers, numberOfCoefficients);
      calculateAlphaPrimeMatrix(generalizedAlphaPrimeRow, numberOfCoefficients, currentTimeDerivativePowers, omegaPowers);
      calculateBetaPrimeMatrix(generalizedBetaPrimeRow, numberOfCoefficients, finalTimeDerivativePowers, omegaPowers, gammaPrime);
      extractPolynomialCoefficients(cmpPolynomial, polynomialCoefficientVector);
      return calculateCapturePoint(generalizedAlphaPrimeRow, generalizedBetaPrimeRow, gammaPrime, polynomialCoefficientVector, icpPositionDesiredFinal);
   }

   private static void extractPolynomialCoefficients(Trajectory trajectory, DenseMatrix64F polynomialCoefficientVector)
   {
      int numberOfCoefficients = trajectory.getNumberOfCoefficients();
      polynomialCoefficientVector.reshape(numberOfCoefficients, 1);
      for (int i = 0; i < numberOfCoefficients; i++)
         polynomialCoefficientVector.set(i, 0, trajectory.getCoefficient(i));
   }

   public void computeDesiredCornerPoints(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                          List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack, List<FrameTrajectory3D> cmpPolynomials3D,
                                          double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);

      cmpPolynomial3D.compute(cmpPolynomial3D.getFinalTime());
      FramePoint3DReadOnly nextEntryCornerPoint = cmpPolynomial3D.getFramePosition();

      for (int i = cmpPolynomials3D.size() - 1; i >= 0; i--)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);

         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         exitCornerPoint.set(nextEntryCornerPoint);
         computeDesiredCapturePointPosition(omega0, cmpPolynomial3D.getInitialTime(), exitCornerPoint, cmpPolynomial3D, entryCornerPoint);
         nextEntryCornerPoint = entryCornerPoint;
      }
   }

   public void computeDesiredCapturePointPosition(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                  FixedFramePoint3DBasics desiredCapturePointToPack)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         double icpPositionDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 0, cmpPolynomial,
                                                                                          finalCapturePoint.getElement(dir.ordinal()));

         desiredCapturePointToPack.setElement(dir.ordinal(), icpPositionDesired);
      }
   }

   public void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                  FixedFrameVector3DBasics desiredCapturePointVelocityToPack)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         double icpVelocityDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 1, cmpPolynomial,
                                                                                          finalCapturePoint.getElement(dir.ordinal()));

         desiredCapturePointVelocityToPack.setElement(dir.ordinal(), icpVelocityDesired);
      }
   }

   public void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                      FixedFrameVector3DBasics desiredCapturePointAccelerationToPack)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         double icpAccelerationDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 2, cmpPolynomial,
                                                                                              finalCapturePoint.getElement(dir.ordinal()));

         desiredCapturePointAccelerationToPack.setElement(dir.ordinal(), icpAccelerationDesired);
      }
   }

   private final List<Double> tempICPQuantitiesList = new ArrayList<>();

   public void computeDesiredCapturePointLinearData(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                    FixedFramePoint3DBasics desiredCapturePointToPack,
                                                    FixedFrameVector3DBasics desiredCapturePointVelocityToPack,
                                                    FixedFrameVector3DBasics desiredCapturePointAccelerationToPack)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         calculateICPQuantitiesFromCorrespondingCMPPolynomial1D(tempICPQuantitiesList, omega0, time, 2, cmpPolynomial,
                                                                finalCapturePoint.getElement(dir.ordinal()));

         desiredCapturePointToPack.setElement(dir.ordinal(), tempICPQuantitiesList.get(0));
         desiredCapturePointVelocityToPack.setElement(dir.ordinal(), tempICPQuantitiesList.get(1));
         desiredCapturePointAccelerationToPack.setElement(dir.ordinal(), tempICPQuantitiesList.get(2));
      }
   }

   public void calculateICPPositionFromCorrespondingCMPPolynomial3D(double omega0, double time, FramePoint3DReadOnly desiredFinalICPPosition,
                                                                    FrameTrajectory3D cmpPolynomials, FixedFramePoint3DBasics desiredICPPositionToPack)
   {
      desiredICPPositionToPack.checkReferenceFrameMatch(desiredFinalICPPosition);
      desiredICPPositionToPack.checkReferenceFrameMatch(cmpPolynomials);
      int maxNumberOfPolynomialCoefficients = cmpPolynomials.getNumberOfCoefficients();
      double finalTime = cmpPolynomials.getFinalTime();
      calculateOmegaPowers(omega0, omegaPowers, maxNumberOfPolynomialCoefficients);
      calculateTimePowers(currentTimeDerivativePowers, time, maxNumberOfPolynomialCoefficients);
      calculateTimePowers(finalTimeDerivativePowers, finalTime, maxNumberOfPolynomialCoefficients);
      double gammaPrime = calculateGammaPrimeMatrix(omega0, time, finalTime);
      calculateAlphaPrimeMatrix(generalizedAlphaPrimeRow, maxNumberOfPolynomialCoefficients, currentTimeDerivativePowers, omegaPowers);
      calculateBetaPrimeMatrix(generalizedBetaPrimeRow, maxNumberOfPolynomialCoefficients, finalTimeDerivativePowers, omegaPowers, gammaPrime);
      for (Axis axis : Axis.values)
      {
         Trajectory axisTrajectory = cmpPolynomials.getTrajectory(axis);
         extractPolynomialCoefficients(axisTrajectory, polynomialCoefficientVector);
         double desiredICP = calculateCapturePoint(generalizedAlphaPrimeRow, generalizedBetaPrimeRow, gammaPrime, polynomialCoefficientVector,
                                                   desiredFinalICPPosition.getElement(axis.ordinal()));
         desiredICPPositionToPack.setElement(axis.ordinal(), desiredICP);
      }
   }

   /**
    * Backward iteration to determine &xi;<sub>ref,&phi;</sub>(0) and &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>) for all segments &phi;
    */
   public void computeDesiredCornerPoints3D(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                            List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack, List<FrameTrajectory3D> cmpPolynomials3D,
                                            double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);
      cmpPolynomial3D.compute(cmpPolynomial3D.getFinalTime());
      FramePoint3DReadOnly nextEntryCornerPoint = cmpPolynomial3D.getFramePosition();

      for (int i = cmpPolynomials3D.size() - 1; i >= 0; i--)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);
         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         exitCornerPoint.set(nextEntryCornerPoint);
         calculateICPPositionFromCorrespondingCMPPolynomial3D(omega0, cmpPolynomial3D.getInitialTime(), exitCornerPoint, cmpPolynomial3D, entryCornerPoint);
         nextEntryCornerPoint = entryCornerPoint;
      }
   }

   // Retained to maintain API
   public void computeDesiredCapturePointPosition3D(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                    FixedFramePoint3DBasics desiredCapturePointToPack)
   {
      calculateICPPositionFromCorrespondingCMPPolynomial3D(omega0, time, finalCapturePoint, cmpPolynomial3D, desiredCapturePointToPack);
   }

   /**
    * Variation of J. Englsberger's "Smooth trajectory generation and push-recovery based on DCM"
    * <br>
    * The approach for calculating DCMs is based on CMP polynomials instead of discrete waypoints
    */
   public void calculateICPQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int icpDerivativeOrder, FrameTrajectory3D cmpPolynomial3D,
                                                                    FrameTuple3DReadOnly icpPositionDesiredFinal,
                                                                    FixedFrameTuple3DBasics icpQuantityDesiredToPack)
   {
      icpQuantityDesiredToPack.checkReferenceFrameMatch(icpPositionDesiredFinal);
      icpQuantityDesiredToPack.checkReferenceFrameMatch(cmpPolynomial3D);
      int maxNumberOfPolynomialCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      double finalTime = cmpPolynomial3D.getFinalTime();
      calculateOmegaPowers(omega0, omegaPowers, maxNumberOfPolynomialCoefficients);
      calculateTimePowers(currentTimeDerivativePowers, time, maxNumberOfPolynomialCoefficients);
      calculateTimePowers(finalTimeDerivativePowers, finalTime, maxNumberOfPolynomialCoefficients);
      double gammaPrime = calculateGammaPrimeMatrix(omega0, time, finalTime);
      calculateAlphaPrimeMatrix(generalizedAlphaPrimeRow, maxNumberOfPolynomialCoefficients, currentTimeDerivativePowers, omegaPowers);
      calculateBetaPrimeMatrix(generalizedBetaPrimeRow, maxNumberOfPolynomialCoefficients, finalTimeDerivativePowers, omegaPowers, gammaPrime);
      for (Axis axis : Axis.values)
      {
         Trajectory axisTrajectory = cmpPolynomial3D.getTrajectory(axis);
         extractPolynomialCoefficients(axisTrajectory, polynomialCoefficientVector);
         double desiredICP = calculateCapturePoint(generalizedAlphaPrimeRow, generalizedBetaPrimeRow, gammaPrime, polynomialCoefficientVector,
                                                   icpPositionDesiredFinal.getElement(axis.ordinal()));
         icpQuantityDesiredToPack.setElement(axis.ordinal(), desiredICP);
      }

   }

   /**
    * Compute the i-th derivative of &xi;<sub>ref,&phi;</sub> at time t<sub>&phi;</sub>: 
    * <P>
    * &xi;<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>) = 
    * (&alpha;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>)
    *  - &beta;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub>
    *  + &gamma;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) * &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    */
   public void calculateICPQuantity3D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, double generalizedGammaPrime,
                                      DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple3DReadOnly icpPositionDesiredFinal,
                                      FixedFrameTuple3DBasics icpQuantityDesiredToPack)
   {
      int numRows = generalizedAlphaBetaPrimeMatrix.getNumRows();
      M1.reshape(numRows, 1);

      icpPositionDesiredFinal.get(M1);
      CommonOps.scale(generalizedGammaPrime, M1);
      CommonOps.multAdd(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientCombinedVector, M1);

      icpQuantityDesiredToPack.set(M1);
   }

   /**
    * Compute the i-th derivative of &alpha;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &alpha;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-j</sup> *
    * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    */
   public void calculateGeneralizedAlphaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaPrimeToPack, int alphaDerivativeOrder,
                                                            FrameTrajectory3D cmpPolynomial3D)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);

         calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrimeRow, alphaDerivativeOrder, cmpPolynomial);

         MatrixTools.setMatrixBlock(generalizedAlphaPrimeToPack, dir.ordinal(), dir.ordinal() * generalizedAlphaPrimeRow.numCols, generalizedAlphaPrimeRow, 0,
                                    0, generalizedAlphaPrimeRow.numRows, generalizedAlphaPrimeRow.numCols, 1.0);
      }
   }

   public static void calculateGeneralizedAlphaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaPrimeRow,
                                                                   int alphaDerivativeOrder, Trajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      generalizedAlphaPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaPrimeRow.zero();

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double scalar = Math.pow(omega0, -i);
         CommonOps.addEquals(generalizedAlphaPrimeRow, scalar, cmpPolynomial.getXPowersDerivativeVector(i + alphaDerivativeOrder, time));
      }
   }

   /**
    * Compute the i-th derivative of &beta;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &beta;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-(j-i)</sup> *
    * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    */
   public void calculateGeneralizedBetaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaPrimeToPack, int betaDerivativeOrder,
                                                           FrameTrajectory3D cmpPolynomial3D)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);

         calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrimeRow, betaDerivativeOrder, cmpPolynomial);

         MatrixTools.setMatrixBlock(generalizedBetaPrimeToPack, dir.ordinal(), dir.ordinal() * generalizedBetaPrimeRow.numCols, generalizedBetaPrimeRow, 0, 0,
                                    generalizedBetaPrimeRow.numRows, generalizedBetaPrimeRow.numCols, 1.0);
      }
   }

   public static void calculateGeneralizedBetaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaPrimeRowToPack,
                                                                  int betaDerivativeOrder, Trajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();

      generalizedBetaPrimeRowToPack.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRowToPack.zero();

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double scalar = Math.pow(omega0, betaDerivativeOrder - i) * Math.exp(omega0 * (time - timeSegmentTotal));
         CommonOps.addEquals(generalizedBetaPrimeRowToPack, scalar, cmpPolynomial.getXPowersDerivativeVector(i, timeSegmentTotal));
      }
   }

   /**
    * Compute the i-th derivative of &gamma;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    */
   public static double calculateGeneralizedGammaPrimeOnCMPSegment3D(double omega0, double time, int gammaDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      return Math.pow(omega0, gammaDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentTotal));
   }

   public static double calculateGeneralizedGammaPrimeOnCMPSegment1D(double omega0, double time, int gammaDerivativeOrder, Trajectory cmpPolynomial)
   {
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      return Math.pow(omega0, gammaDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentTotal));
   }

   public static double calculateGeneralizedMatricesPrimeOnCMPSegment1D(double omega0, double time, int derivativeOrder, Trajectory cmpPolynomial,
                                                                        DenseMatrix64F generalizedAlphaPrime, DenseMatrix64F generalizedBetaPrime,
                                                                        DenseMatrix64F generalizedAlphaBetaPrime)
   {
      calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrime, derivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrime, derivativeOrder, cmpPolynomial);
      double generalizedGammaPrime = calculateGeneralizedGammaPrimeOnCMPSegment1D(omega0, time, derivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaPrime, generalizedBetaPrime, generalizedAlphaBetaPrime);

      return generalizedGammaPrime;
   }

   private void initializeMatrices3D(int numberOfCoefficients)
   {
      initializeMatrices(3, numberOfCoefficients);
   }

   private void initializeMatrices(int dimension, int numberOfCoefficients)
   {
      polynomialCoefficientCombinedVector.reshape(dimension * numberOfCoefficients, 1);
      polynomialCoefficientCombinedVector.zero();

      generalizedAlphaPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaPrimeMatrix.zero();

      generalizedBetaPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedBetaPrimeMatrix.zero();

      generalizedAlphaBetaPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaBetaPrimeMatrix.zero();
   }

   private void setPolynomialCoefficientVector3D(DenseMatrix64F polynomialCoefficientCombinedVectorToPack, FrameTrajectory3D cmpPolynomial3D)
   {
      int numRows = cmpPolynomial3D.getNumberOfCoefficients();
      int numCols = 1;
      for (Axis dir : Axis.values)
      {
         setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial3D.getTrajectory(dir));
         MatrixTools.setMatrixBlock(polynomialCoefficientCombinedVectorToPack, dir.ordinal() * numRows, 0, polynomialCoefficientVector, 0, 0, numRows, numCols,
                                    1.0);
      }
   }

   private static void setPolynomialCoefficientVector1D(DenseMatrix64F polynomialCoefficientVectorToPack, Trajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();

      polynomialCoefficientVectorToPack.setData(polynomialCoefficients);
      polynomialCoefficientVectorToPack.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
   }
}
