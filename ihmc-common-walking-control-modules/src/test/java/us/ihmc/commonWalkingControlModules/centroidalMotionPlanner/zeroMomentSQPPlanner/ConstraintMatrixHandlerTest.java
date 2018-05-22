package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstraintMatrixHandlerTest
{
   private YoVariableRegistry registry;
   private ConstraintMatrixHandler handler;
   private YoInteger numberOfSegments;
   private YoInteger numberOfCoMCoefficients;
   private YoInteger numberOfCoPCoefficients;
   private YoInteger numberOfScalarCoefficients;

   @Test(timeout = 100000)
   public void testInitialization()
   {
      initialize();
      assertTrue(handler != null);
   }

   private void initialize()
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      numberOfSegments = new YoInteger("NumberOfSegments", registry);
      numberOfSegments.set(4);
      numberOfCoMCoefficients = new YoInteger("NumberOfCoMCoefficients", registry);
      numberOfCoMCoefficients.set(4);
      numberOfCoPCoefficients = new YoInteger("NumberOfCoPCoefficients", registry);
      numberOfCoPCoefficients.set(4);
      numberOfScalarCoefficients = new YoInteger("NumberOfScalarCoefficients", registry);
      numberOfScalarCoefficients.set(2);
      handler = new ConstraintMatrixHandler(numberOfSegments, numberOfCoMCoefficients, numberOfCoPCoefficients, numberOfScalarCoefficients);
      handler.reshape();
   }

   @Test(timeout = 100000)
   public void testReshape()
   {
      initialize();
      testInterSegmentCoPConstraintHandling();
      testIntraSegmentCoMConstraintHandling();
      testDynamicsConstraintHandling();
      numberOfSegments.set(11);
      numberOfCoMCoefficients.set(5);
      numberOfCoPCoefficients.set(8);
      numberOfScalarCoefficients.set(2);
      handler.reshape();
      assertTrue(handler.getCoefficientMatrix().numCols == 11 * 33);
   }

   @Test(timeout = 100000)
   public void testInterSegmentCoPConstraintHandling()
   {
      initialize();
      insertInterSegmentCoPConstraint();
   }

   private void insertInterSegmentCoPConstraint()
   {
      Random random = new Random(1234152l);
      int numRows = 1;
      DenseMatrix64F coefficient1 = new DenseMatrix64F(numRows, numberOfCoPCoefficients.getIntegerValue());
      DenseMatrix64F bias1 = new DenseMatrix64F(numRows, 1);
      DenseMatrix64F coefficient2 = new DenseMatrix64F(numRows, numberOfCoPCoefficients.getIntegerValue());
      DenseMatrix64F bias2 = new DenseMatrix64F(numRows, 1);
      for(int i = 0; i < numberOfCoPCoefficients.getIntegerValue(); i++)
      {
         coefficient1.set(0, i, random.nextInt(10));
         coefficient2.set(0, i, random.nextInt(10));
      }
      bias1.set(0, 0, random.nextInt(10));
      bias2.set(0, 0, random.nextInt(10));
      handler.addInterSegmentCoPConstraint(Axis.X, 2, coefficient1, bias1, 3, coefficient2, bias2);
      DenseMatrix64F A = handler.getCoefficientMatrix();
      DenseMatrix64F b = handler.getBiasMatrix();
      int cols = 3 * numberOfCoMCoefficients.getIntegerValue() + 2 * numberOfCoPCoefficients.getIntegerValue() + numberOfScalarCoefficients.getIntegerValue();
      for(int i = 0; i < numberOfSegments.getIntegerValue(); i++)
      {
         String str = "";
         for (int j = 0; j < cols; j++)
            str += " " + String.format("%+4.3f", A.get(0, i * cols + j));
         PrintTools.debug(str);
      }
      PrintTools.debug(coefficient1.toString());
      PrintTools.debug(coefficient2.toString());
      PrintTools.debug("Bias " + b.toString());
      PrintTools.debug(bias1.toString());
      PrintTools.debug(bias2.toString());
   }

   @Test(timeout = 100000)
   public void testIntraSegmentCoPConstraintHandling()
   {
      assertTrue(false);

   }

   @Test(timeout = 100000)
   public void testInterSegmentCoMConstraintHandling()
   {
      assertTrue(false);

   }

   @Test(timeout = 100000)
   public void testIntraSegmentCoMConstraintHandling()
   {
      assertTrue(false);

   }

   @Test(timeout = 100000)
   public void testInterSegmentScalarConstraintHandling()
   {
      assertTrue(false);

   }

   @Test(timeout = 100000)
   public void testIntraSegmentScalarConstraintHandling()
   {
      assertTrue(false);

   }

   @Test(timeout = 100000)
   public void testDynamicsConstraintHandling()
   {
      assertTrue(false);

   }
}
