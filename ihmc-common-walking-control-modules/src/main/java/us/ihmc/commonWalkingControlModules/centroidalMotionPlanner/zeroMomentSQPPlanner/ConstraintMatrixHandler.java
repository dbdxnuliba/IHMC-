package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstraintMatrixHandler
{
   private static final int numberOfCoMAxis = 3;
   private static final int numberOfCoPAxis = 2;
   private final YoInteger numberOfSegments;
   private final YoInteger numberOfCoMCoefficients;
   private final YoInteger numberOfCoPCoefficients;
   private final YoInteger numberOfScalarCoefficients;

   private final DenseMatrix64F A;
   private final DenseMatrix64F b;
   private final DenseMatrix64F tempMatrix;
   private int numCols;
   private int colsPerSegment;
   private int comOffset;
   private int copOffset;
   private int scalarOffset;

   public ConstraintMatrixHandler(YoInteger numberOfSegments, YoInteger numberOfCoMCoefficients, YoInteger numberOfCoPCoefficients,
                                  YoInteger numberOfScalarCoefficients)
   {
      this.numberOfSegments = numberOfSegments;
      this.numberOfCoMCoefficients = numberOfCoMCoefficients;
      this.numberOfCoPCoefficients = numberOfCoPCoefficients;
      this.numberOfScalarCoefficients = numberOfScalarCoefficients;

      A = new DenseMatrix64F(300, 1000);
      b = new DenseMatrix64F(300, 1);
      tempMatrix = new DenseMatrix64F(0, 1);
   }

   public void reshape()
   {
      colsPerSegment = numberOfCoMAxis * numberOfCoMCoefficients.getIntegerValue() + numberOfCoPAxis * numberOfCoPCoefficients.getIntegerValue()
            + numberOfScalarCoefficients.getIntegerValue();
      comOffset = 0;
      copOffset = numberOfCoMAxis * numberOfCoMCoefficients.getIntegerValue();
      scalarOffset = copOffset + numberOfCoPAxis * numberOfCoPCoefficients.getIntegerValue();
      numCols = numberOfSegments.getIntegerValue() * colsPerSegment;
      A.reshape(0, numCols);
      b.reshape(0, 1);
   }

   public void addInterSegmentScalarConstraint(int segmentIndex1, DenseMatrix64F segment1CoefficientMatrix, DenseMatrix64F segment1BiasMatrix,
                                               int segmentIndex2, DenseMatrix64F segment2CoefficientMatrix, DenseMatrix64F segment2BiasMatrix)
   {
      if (segment1CoefficientMatrix.getNumRows() != segment1BiasMatrix.getNumRows() || segment2CoefficientMatrix.getNumRows() != segment2BiasMatrix.getNumRows()
            || segment1CoefficientMatrix.getNumRows() != segment2CoefficientMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismatch in the specified constraint coefficient and bias matrices");
      if (segment1CoefficientMatrix.getNumCols() != numberOfScalarCoefficients.getIntegerValue()
            || segment2CoefficientMatrix.getNumCols() != numberOfScalarCoefficients.getIntegerValue() || segment1BiasMatrix.getNumCols() != 1
            || segment2BiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns do not match the number of coefficients");

      tempMatrix.reshape(segment1CoefficientMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int colIndexToInsertSegment1 = segmentIndex1 * colsPerSegment + scalarOffset;
      CommonOps.insert(segment1CoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment1);
      CommonOps.scale(-1.0, tempMatrix);
      int colIndexToInsertSegment2 = segmentIndex2 * colsPerSegment + scalarOffset;
      CommonOps.insert(segment2CoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment2);
      int numberOfRows = A.getNumRows();
      A.reshape(numberOfRows + tempMatrix.getNumRows(), numCols, true);
      CommonOps.insert(tempMatrix, A, numberOfRows, 0);

      tempMatrix.reshape(segment1BiasMatrix.getNumRows(), 1);
      tempMatrix.zero();
      CommonOps.subtractEquals(tempMatrix, segment1BiasMatrix);
      CommonOps.addEquals(tempMatrix, segment2BiasMatrix);
      b.reshape(numberOfRows + tempMatrix.getNumRows(), 1, true);
      CommonOps.insert(tempMatrix, b, numberOfRows, 0);
   }

   public void addInterSegmentCoPConstraint(Axis axis, int segmentIndex1, DenseMatrix64F segment1CoefficientMatrix, DenseMatrix64F segment1BiasMatrix,
                                            int segmentIndex2, DenseMatrix64F segment2CoefficientMatrix, DenseMatrix64F segment2BiasMatrix)
   {
      if (axis == Axis.Z)
         throw new RuntimeException("Z axis CoP not supported");
      if (segment1CoefficientMatrix.getNumRows() != segment1BiasMatrix.getNumRows() || segment2CoefficientMatrix.getNumRows() != segment2BiasMatrix.getNumRows()
            || segment1CoefficientMatrix.getNumRows() != segment2CoefficientMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismatch in the specified constraint coefficient and bias matrices");
      if (segment1CoefficientMatrix.getNumCols() != numberOfCoPCoefficients.getIntegerValue()
            || segment2CoefficientMatrix.getNumCols() != numberOfCoPCoefficients.getIntegerValue() || segment1BiasMatrix.getNumCols() != 1
            || segment2BiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns do not match the number of coefficeints");

      tempMatrix.reshape(segment1CoefficientMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int axisSpecificOffset = copOffset + axis.ordinal() * numberOfCoPCoefficients.getIntegerValue();
      int colIndexToInsertSegment1 = segmentIndex1 * colsPerSegment + axisSpecificOffset;
      CommonOps.insert(segment1CoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment1);
      CommonOps.scale(-1.0, tempMatrix);
      int colIndexToInsertSegment2 = segmentIndex2 * colsPerSegment + axisSpecificOffset;
      CommonOps.insert(segment2CoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment2);
      int numberOfRows = A.getNumRows();
      A.reshape(numberOfRows + tempMatrix.getNumRows(), numCols, true);
      CommonOps.insert(tempMatrix, A, numberOfRows, 0);

      tempMatrix.reshape(segment1BiasMatrix.getNumRows(), 1);
      tempMatrix.zero();
      CommonOps.subtractEquals(tempMatrix, segment1BiasMatrix);
      CommonOps.addEquals(tempMatrix, segment2BiasMatrix);
      b.reshape(numberOfRows + tempMatrix.getNumRows(), 1, true);
      CommonOps.insert(tempMatrix, b, numberOfRows, 0);
   }

   public void addInterSegmentCoMConstraint(Axis axis, int segmentIndex1, DenseMatrix64F segment1CoefficientMatrix, DenseMatrix64F segment1BiasMatrix,
                                            int segmentIndex2, DenseMatrix64F segment2CoefficientMatrix, DenseMatrix64F segment2BiasMatrix)
   {
      if (segment1CoefficientMatrix.getNumRows() != segment1BiasMatrix.getNumRows() || segment2CoefficientMatrix.getNumRows() != segment2BiasMatrix.getNumRows()
            || segment1CoefficientMatrix.getNumRows() != segment2CoefficientMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismatch in the specified constraint coefficient and bias matrices");
      if (segment1CoefficientMatrix.getNumCols() != numberOfCoMCoefficients.getIntegerValue()
            || segment2CoefficientMatrix.getNumCols() != numberOfCoMCoefficients.getIntegerValue() || segment1BiasMatrix.getNumCols() != 1
            || segment2BiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns do not match the number of coefficients");

      tempMatrix.reshape(segment1CoefficientMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int axisSpecificOffset = comOffset + axis.ordinal() * numberOfCoMCoefficients.getIntegerValue();
      int colIndexToInsertSegment1 = segmentIndex1 * colsPerSegment + axisSpecificOffset;
      CommonOps.insert(segment1CoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment1);
      CommonOps.scale(-1.0, tempMatrix);
      int colIndexToInsertSegment2 = segmentIndex2 * colsPerSegment + axisSpecificOffset;
      CommonOps.insert(segment2CoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment2);
      int numberOfRows = A.getNumRows();
      A.reshape(numberOfRows + tempMatrix.getNumRows(), numCols, true);
      CommonOps.insert(tempMatrix, A, numberOfRows, 0);

      tempMatrix.reshape(segment1BiasMatrix.getNumRows(), 1);
      tempMatrix.zero();
      CommonOps.subtractEquals(tempMatrix, segment1BiasMatrix);
      CommonOps.addEquals(tempMatrix, segment2BiasMatrix);
      b.reshape(numberOfRows + tempMatrix.getNumRows(), 1, true);
      CommonOps.insert(tempMatrix, b, numberOfRows, 0);
   }

   public void addIntraSegmentScalarConstraint(int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public void addIntraSegmentCoPConstraint(Axis axis, int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public void addIntraSegmentCoMConstraint(Axis axis, int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public void addIntraSegmentConstraint(int segmentIndex, DenseMatrix64F segmentCoefficientMatrix, DenseMatrix64F segmentBiasMatrix)
   {

   }

   public DenseMatrix64F getCoefficientMatrix()
   {
      return A;
   }

   public DenseMatrix64F getBiasMatrix()
   {
      return b;
   }
}