package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

/**
 * 
 * @author Apoorv S
 *
 */
public class ConstraintGenerationHelper
{
   private final DenseMatrix64F xPowers = new DenseMatrix64F(0, 1);

   public ConstraintGenerationHelper()
   {

   }

   private void computePowers(DenseMatrix64F powersMatrix, double x, int polynomialOrder)
   {
      powersMatrix.reshape(polynomialOrder + 1, 1);
      powersMatrix.zero();
      powersMatrix.set(0, 0, 1.0);
      for (int i = 1; i <= polynomialOrder; i++)
         powersMatrix.set(i, 0, powersMatrix.get(i - 1, 0) * x);
   }

   public void generateCoefficients(DenseMatrix64F constraintCoefficentToSet, int polynomialOrder, int derivatiesOrder, double x)
   {
      constraintCoefficentToSet.reshape(derivatiesOrder + 1, polynomialOrder + 1);
      constraintCoefficentToSet.zero();
      computePowers(xPowers, x, polynomialOrder);
      for (int i = 0; i <= polynomialOrder; i++)
      {
         double fact = 1.0;
         for (int j = 0; j <= derivatiesOrder && j <= i; j++)
         {
            constraintCoefficentToSet.set(j, i, fact * xPowers.get(i - j, 0));
            fact *= (i - j);
         }
      }
   }

   /**
    * 
    * @param constraintCoefficentsToSet modified.
    * @param constraintBiasToSet modified
    * @param polynomialCoefficient column vector of polynomial coefficients
    * @param polynomialOrder order of the polynomial 
    * @param derivatiesOrder order of the derivatives to be determined
    */
   public void generateDerivativeCoefficientsAndBiasMatrix(DenseMatrix64F constraintCoefficentsToSet, DenseMatrix64F constraintBiasToSet,
                                                           DenseMatrix64F polynomialCoefficient, int polynomialOrder, int derivatiesOrder, double x)
   {
      generateCoefficients(constraintCoefficentsToSet, polynomialOrder, derivatiesOrder, x);
      constraintBiasToSet.reshape(derivatiesOrder + 1, 1);
      CommonOps.mult(constraintCoefficentsToSet, polynomialCoefficient, constraintBiasToSet);
   }

   public void generateLocationConstraintMatrix()
   {

   }

   private final DenseMatrix64F tempCoeffMatrixForCoM = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempCoeffMatrixForCoP = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempCoeffMatrixForScalar = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempBiasMatrixForCoM = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempBiasMatrixForCoP = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempBiasMatrixForScalar = new DenseMatrix64F(0, 1);

   public void generateDynamicsCollocationConstraints(DenseMatrix64F comCoefficientMatrixToSet, DenseMatrix64F copCoefficientMatrixToSet,
                                                      DenseMatrix64F scalarCoefficientMatrixToSet, DenseMatrix64F biasMatrixToSet,
                                                      List<Double> timeNodesForConstraints, DenseMatrix64F comCoefficients, DenseMatrix64F copCoefficients,
                                                      DenseMatrix64F scalarCoefficients, int comPolynomialOrder, int copPolynomialOrder,
                                                      int scalarPolynomialOrder)
   {
      comCoefficientMatrixToSet.reshape(timeNodesForConstraints.size(), comPolynomialOrder + 1);
      copCoefficientMatrixToSet.reshape(timeNodesForConstraints.size(), copPolynomialOrder + 1);
      scalarCoefficientMatrixToSet.reshape(timeNodesForConstraints.size(), scalarPolynomialOrder + 1);

      for (int i = 0; i < timeNodesForConstraints.size(); i++)
      {
         double nodeTime = timeNodesForConstraints.get(i);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrixForCoM, tempBiasMatrixForCoM, comCoefficients, comPolynomialOrder, 0, nodeTime);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrixForCoP, tempBiasMatrixForCoP, copCoefficients, copPolynomialOrder, 0, nodeTime);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrixForScalar, tempBiasMatrixForScalar, scalarCoefficients, scalarPolynomialOrder, 0, nodeTime);
         for (int j = comPolynomialOrder; j >= 2; j--)
            tempCoeffMatrixForCoM.set(0, j, tempCoeffMatrixForCoM.get(0, j - 2) * j * (j - 1) - tempBiasMatrixForScalar.get(0, 0) * tempCoeffMatrixForCoM.get(0, j));
         for (int j = 1; j >= 0; j--)
            tempCoeffMatrixForCoM.set(0, j, -tempBiasMatrixForScalar.get(0, 0) * tempCoeffMatrixForCoM.get(0, j));
         CommonOps.scale(tempBiasMatrixForScalar.get(0, 0), tempCoeffMatrixForCoP);
         CommonOps.scale(tempBiasMatrixForCoP.get(0, 0) - tempBiasMatrixForCoM.get(0, 0), tempCoeffMatrixForScalar);
         
         CommonOps.insert(tempCoeffMatrixForCoM, comCoefficientMatrixToSet, i, 0);
         CommonOps.insert(tempCoeffMatrixForCoP, copCoefficientMatrixToSet, i, 0);
         CommonOps.insert(tempCoeffMatrixForScalar, scalarCoefficientMatrixToSet, i, 0);
      }
   }
}
