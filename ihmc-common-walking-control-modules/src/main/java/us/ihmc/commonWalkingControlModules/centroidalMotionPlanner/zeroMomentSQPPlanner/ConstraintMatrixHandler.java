package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.Axis;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstraintMatrixHandler
{
   private final int numberOfCoMAxis = 3;
   private final int numberOfCoPAxis = 2;
   private final YoInteger numberOfSegments;
   private final int numberOfCoMCoefficients;
   private final int numberOfCoPCoefficients;
   private final int numberOfScalarCoefficents;

   public ConstraintMatrixHandler(YoInteger numberOfSegments, int numberOfCoMCoefficients, int numberOfCoPCoefficients, int numberOfScalarCoefficients)
   {
      this.numberOfSegments = numberOfSegments;
      this.numberOfCoMCoefficients = numberOfCoMCoefficients;
      this.numberOfCoPCoefficients = numberOfCoPCoefficients;
      this.numberOfScalarCoefficents = numberOfScalarCoefficients;
   }

   public void reset()
   {
      
   }
   
   public void processInterSegmentScalarConstraint(int segmentIndex1, DenseMatrix64F segment1CoefficientMatrix, DenseMatrix64F segment1BiasMatrix,
                                               int segmentIndex2, DenseMatrix64F segment2CoefficientMatrix, DenseMatrix64F segment2BiasMatrix)
   {
      
   }

   public void processInterSegmentCoPConstraint(Axis axis, int segmentIndex1, DenseMatrix64F segment1CoefficientMatrix, DenseMatrix64F segment1BiasMatrix,
                                            int segmentIndex2, DenseMatrix64F segment2CoefficientMatrix, DenseMatrix64F segment2BiasMatrix)
   {
      
   }

   public void processInterSegmentCoMConstraint(Axis axis, int segmentIndex1, DenseMatrix64F segment1CoefficientMatrix, DenseMatrix64F segment1BiasMatrix,
                                            int segmentIndex2, DenseMatrix64F segment2CoefficientMatrix, DenseMatrix64F segment2BiasMatrix)
   {

   }

   public void processIntraSegmentScalarConstraint(int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public void processIntraSegmentCoPConstraint(Axis axis, int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public void processIntraSegmentCoMConstraint(Axis axis, int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public void processIntraSegmentConstraint(int segmentIndex, DenseMatrix64F segmentCoefficientMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }
}