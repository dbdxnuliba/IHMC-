package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.math.trajectories.Trajectory;

/**
 * Generates constraints that ensure N-th order smoothness between two polynomials 
 * of order M
 * @author Apoorv
 *
 */
public class ConstraintGenerator
{
   private final ConstraintMatrixHandler constraintHandler;
   private final DenseMatrix64F xPowers = new DenseMatrix64F(0, 1);

   public ConstraintGenerator(ConstraintMatrixHandler constraintHandler)
   {
      this.constraintHandler = constraintHandler;
   }
   
   private void computePowers(DenseMatrix64F powersMatrix, double x, int polynomialOrder)
   {
      powersMatrix.reshape(polynomialOrder + 1, 1);
      powersMatrix.zero();
      powersMatrix.set(0, 0, 1.0);
      for (int i = 1; i <= polynomialOrder; i++)
         powersMatrix.set(i, 0, powersMatrix.get(i - 1, 0) * x);
   }

   private void generateCoefficients(DenseMatrix64F constraintCoefficentToSet, int polynomialOrder, int derivatiesOrder, double x)
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
   private void generateDerivativeCoefficientsAndBiasMatrix(DenseMatrix64F constraintCoefficentsToSet, DenseMatrix64F constraintBiasToSet, DenseMatrix64F polynomialCoefficient,
                                           int polynomialOrder, int derivatiesOrder, double x)
   {
      generateCoefficients(constraintCoefficentsToSet, polynomialOrder, derivatiesOrder, x);
      constraintBiasToSet.reshape(derivatiesOrder + 1, 1);
      CommonOps.mult(constraintCoefficentsToSet, polynomialCoefficient, constraintBiasToSet);
   }

   public void generateScalarConstraintsFromContactStates(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      // TODO Auto-generated method stub

   }

   public void generateCoMLocationConstraintsFromContactStates(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      // TODO Auto-generated method stub

   }

   public void generateCoPLocationConstraintsFromContactStates(List<CollinearForceMotionPlannerSegment> segmentList)
   {
      // TODO Auto-generated method stub

   }


}
