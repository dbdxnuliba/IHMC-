package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;

import javax.management.RuntimeErrorException;
import javax.xml.bind.SchemaOutputResolver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.robotics.lists.RecyclingArrayList;
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
      if (segmentCoefficentMatrix.getNumRows() != segmentBiasMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismath in the specified constraint coefficient and bias matrices");
      if (segmentCoefficentMatrix.getNumCols() != numberOfScalarCoefficients.getIntegerValue() || segmentBiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns for specified coefficient or bias matrix do not match their expected value");

      tempMatrix.reshape(segmentCoefficentMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int colIndexToInsertSegment = segmentIndex * colsPerSegment + scalarOffset;
      CommonOps.insert(segmentCoefficentMatrix, tempMatrix, 0, colIndexToInsertSegment);
      insertRowsIntoConstraintMatrix(tempMatrix, segmentBiasMatrix);
   }

   public void addIntraSegmentCoPConstraint(Axis axis, int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {
      if (segmentCoefficentMatrix.getNumRows() != segmentBiasMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismath in the specified constraint coefficient and bias matrices");
      if (segmentCoefficentMatrix.getNumCols() != numberOfCoPCoefficients.getIntegerValue() || segmentBiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns for specified coefficient or bias matrix do not match their expected value");

      tempMatrix.reshape(segmentCoefficentMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int colIndexToInsertSegment = segmentIndex * colsPerSegment + copOffset + axis.ordinal() * numberOfCoPCoefficients.getIntegerValue();
      CommonOps.insert(segmentCoefficentMatrix, tempMatrix, 0, colIndexToInsertSegment);
      insertRowsIntoConstraintMatrix(tempMatrix, segmentBiasMatrix);
   }

   public void addIntraSegmentCoMConstraint(Axis axis, int segmentIndex, DenseMatrix64F segmentCoefficentMatrix, DenseMatrix64F segmentBiasMatrix)
   {
      if (segmentCoefficentMatrix.getNumRows() != segmentBiasMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismath in the specified constraint coefficient and bias matrices");
      if (segmentCoefficentMatrix.getNumCols() != numberOfCoMCoefficients.getIntegerValue() || segmentBiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns for specified coefficient or bias matrix do not match their expected value");

      tempMatrix.reshape(segmentCoefficentMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int colIndexToInsertSegment = segmentIndex * colsPerSegment + comOffset + axis.ordinal() * numberOfCoMCoefficients.getIntegerValue();
      CommonOps.insert(segmentCoefficentMatrix, tempMatrix, 0, colIndexToInsertSegment);
      insertRowsIntoConstraintMatrix(tempMatrix, segmentBiasMatrix);
   }

   public void addIntraSegmentMultiQuantityConstraints(Axis axis, int segmentIndex, DenseMatrix64F comCoefficients, DenseMatrix64F copCoefficients,
                                                       DenseMatrix64F scalarCoefficients, DenseMatrix64F bias)
   {
      if (axis == Axis.Z)
         throw new RuntimeException("Invalid axis specified. For Z axis use the dedicated method");
      if (comCoefficients.getNumRows() != copCoefficients.getNumRows() || comCoefficients.getNumRows() != scalarCoefficients.getNumRows()
            || comCoefficients.getNumRows() != bias.getNumRows())
         throw new RuntimeException("Numbers of rows mismatch between specified constraint coefficient and / or bias matrices");
      if (comCoefficients.getNumCols() != numberOfCoMCoefficients.getIntegerValue())
         throw new RuntimeException("Number of columns for specified constraints matrices (" + comCoefficients.getNumCols()
               + ") does not match their expected value (" + numberOfCoMCoefficients.getIntegerValue() + ").");
      else if (copCoefficients.getNumCols() != numberOfCoPCoefficients.getIntegerValue())
         throw new RuntimeException("Number of columns for specified constraints matrices (" + copCoefficients.getNumCols()
         + ") does not match their expected value (" + numberOfCoPCoefficients.getIntegerValue() + ").");
      else if (scalarCoefficients.getNumCols() != numberOfScalarCoefficients.getIntegerValue())
         throw new RuntimeException("Number of columns for specified constraints matrices (" + scalarCoefficients.getNumCols()
         + ") does not match their expected value (" + numberOfScalarCoefficients.getIntegerValue() + ").");
      else if (bias.getNumCols() != 1)
         throw new RuntimeException("Number of columns for specified constraints matrices (" + bias.getNumCols()
         + ") does not match their expected value (" + 1 + ").");

      tempMatrix.reshape(copCoefficients.getNumRows(), numCols);
      tempMatrix.zero();

      int segmentColsStart = segmentIndex * colsPerSegment;
      int comInsertionIndex = segmentColsStart + comOffset + axis.ordinal() * numberOfCoMCoefficients.getIntegerValue();
      int copInsertionIndex = segmentColsStart + copOffset + axis.ordinal() * numberOfCoPCoefficients.getIntegerValue();
      int scalarInsertionIndex = segmentColsStart + scalarOffset;
      CommonOps.insert(comCoefficients, tempMatrix, 0, comInsertionIndex);
      CommonOps.insert(copCoefficients, tempMatrix, 0, copInsertionIndex);
      CommonOps.insert(scalarCoefficients, tempMatrix, 0, scalarInsertionIndex);
      insertRowsIntoConstraintMatrix(tempMatrix, bias);
   }

   public void addIntraSegmentMultiQuantityConstraintsForZAxis(int segmentIndex, DenseMatrix64F comCoefficients, DenseMatrix64F scalarCoefficients,
                                                               DenseMatrix64F bias)
   {
      if (comCoefficients.getNumRows() != scalarCoefficients.getNumRows() || comCoefficients.getNumRows() != bias.getNumRows())
         throw new RuntimeException("Numbers of rows mismatch between specified constraint coefficient and / or bias matrices");
      if (comCoefficients.getNumCols() != numberOfCoMCoefficients.getIntegerValue()
            || scalarCoefficients.getNumCols() != numberOfScalarCoefficients.getIntegerValue() || bias.getNumCols() != 1)
         throw new RuntimeException("Number of columns for specified constraints matrices do not match their expected value");

      tempMatrix.reshape(comCoefficients.getNumRows(), numCols);
      tempMatrix.zero();

      int segmentColsStart = segmentIndex * colsPerSegment;
      int comInsertionIndex = segmentColsStart + comOffset + Axis.Z.ordinal() * numberOfCoMCoefficients.getIntegerValue();
      int scalarInsertionIndex = segmentColsStart + scalarOffset;
      CommonOps.insert(comCoefficients, tempMatrix, 0, comInsertionIndex);
      CommonOps.insert(scalarCoefficients, tempMatrix, 0, scalarInsertionIndex);
      insertRowsIntoConstraintMatrix(tempMatrix, bias);
   }

   private void insertRowsIntoConstraintMatrix(DenseMatrix64F constraintCoefficients, DenseMatrix64F biasMatrix)
   {
      int numberOfRows = A.getNumRows();
      A.reshape(numberOfRows + constraintCoefficients.getNumRows(), numCols, true);
      CommonOps.insert(constraintCoefficients, A, numberOfRows, 0);

      b.reshape(numberOfRows + biasMatrix.getNumRows(), 1, true);
      CommonOps.insert(biasMatrix, b, numberOfRows, 0);
   }

   public void addIntraSegmentConstraint(int segmentIndex, DenseMatrix64F segmentCoefficientMatrix, DenseMatrix64F segmentBiasMatrix)
   {
      if (segmentCoefficientMatrix.getNumRows() != segmentBiasMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismath in the specified constraint coefficient and bias matrices");
      if (segmentCoefficientMatrix.getNumCols() != colsPerSegment || segmentBiasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns for specified coefficient or bias matrix do not match their expected value");

      tempMatrix.reshape(segmentCoefficientMatrix.getNumRows(), numCols);
      tempMatrix.zero();
      int colIndexToInsertSegment = segmentIndex * colsPerSegment;
      CommonOps.insert(segmentCoefficientMatrix, tempMatrix, 0, colIndexToInsertSegment);
      insertRowsIntoConstraintMatrix(tempMatrix, segmentBiasMatrix);
   }

   public DenseMatrix64F getCoefficientMatrix()
   {
      return A;
   }

   public DenseMatrix64F getBiasMatrix()
   {
      return b;
   }

   public void addIntraSegmentMultiAxisCoPConstraint(int segmentIndex, DenseMatrix64F xAxisConstraintCoefficient, DenseMatrix64F yAxisConstraintCoefficient,
                                                     DenseMatrix64F biasMatrix)
   {
      if (xAxisConstraintCoefficient.getNumRows() != yAxisConstraintCoefficient.getNumRows()
            || xAxisConstraintCoefficient.getNumRows() != biasMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismatch between specified constraint coefficients and biases");
      if (xAxisConstraintCoefficient.getNumCols() != numberOfCoPCoefficients.getIntegerValue()
            || yAxisConstraintCoefficient.getNumCols() != numberOfCoPCoefficients.getIntegerValue() || biasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns of specified constraint coefficient and bias matrices does not match expected value");

      tempMatrix.reshape(xAxisConstraintCoefficient.getNumRows(), numCols);
      tempMatrix.zero();

      CommonOps.insert(xAxisConstraintCoefficient, tempMatrix, 0,
                       segmentIndex * colsPerSegment + copOffset + Axis.X.ordinal() * numberOfCoPCoefficients.getIntegerValue());
      CommonOps.insert(yAxisConstraintCoefficient, tempMatrix, 0,
                       segmentIndex * colsPerSegment + copOffset + Axis.Y.ordinal() * numberOfCoPCoefficients.getIntegerValue());
      insertRowsIntoConstraintMatrix(tempMatrix, biasMatrix);
   }

   public void addIntraSegmentMultiAxisCoMXYConstraint(int segmentIndex, DenseMatrix64F xAxisConstraintCoefficient, DenseMatrix64F yAxisConstraintCoefficient,
                                                       DenseMatrix64F biasMatrix)
   {
      if (xAxisConstraintCoefficient.getNumRows() != xAxisConstraintCoefficient.getNumRows()
            || xAxisConstraintCoefficient.getNumRows() != biasMatrix.getNumRows())
         throw new RuntimeException("Number of rows mismatch between specified constraint coefficients and biases");
      if (xAxisConstraintCoefficient.getNumCols() != numberOfCoMCoefficients.getIntegerValue()
            || yAxisConstraintCoefficient.getNumCols() != numberOfCoMCoefficients.getIntegerValue() || biasMatrix.getNumCols() != 1)
         throw new RuntimeException("Number of columns of specified constraint coefficient and bias matrices does not match expected value");

      tempMatrix.reshape(xAxisConstraintCoefficient.getNumRows(), numCols);
      tempMatrix.zero();

      CommonOps.insert(xAxisConstraintCoefficient, tempMatrix, 0,
                       segmentIndex * colsPerSegment + comOffset + Axis.X.ordinal() * numberOfCoMCoefficients.getIntegerValue());
      CommonOps.insert(yAxisConstraintCoefficient, tempMatrix, 0,
                       segmentIndex * colsPerSegment + comOffset + Axis.Y.ordinal() * numberOfCoMCoefficients.getIntegerValue());
      insertRowsIntoConstraintMatrix(tempMatrix, biasMatrix);
   }

}