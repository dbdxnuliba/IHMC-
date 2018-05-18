package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class ConstraintGenerationHelperTest
{
   @Test
   public void testSupportPolygonConstraintGeneration()
   {
      CollinearForcePlannerOptimizationControlModuleHelper helper = new CollinearForcePlannerOptimizationControlModuleHelper();
      List<Double> nodeTimes = new ArrayList<>();
      nodeTimes.add(0.0);
      nodeTimes.add(0.15);
      int copTrajectoryOrder = 7;
      DenseMatrix64F xAxisConstraintCoefficientToSet = new DenseMatrix64F(0, 1);
      DenseMatrix64F yAxisConstraintCoefficientToSet = new DenseMatrix64F(0, 1);
      DenseMatrix64F biasMatrixToSet = new DenseMatrix64F(0, 1);
      ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
      supportPolygon.addVertex(-0.1, -0.1);
      supportPolygon.addVertex(-0.1, 0.1);
      supportPolygon.addVertex(0.1, -0.1);
      supportPolygon.addVertex(0.1, 0.1);
      supportPolygon.update();
      Trajectory tempTrajectory = new Trajectory(copTrajectoryOrder + 1);
      DenseMatrix64F xAxisTrajectoryCoefficients = new DenseMatrix64F(copTrajectoryOrder + 1, 1);
      tempTrajectory.setSeptic(0.0, 0.05, 0.10, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      tempTrajectory.getCoefficientVector(xAxisTrajectoryCoefficients);
      DenseMatrix64F yAxisTrajectoryCoefficients = new DenseMatrix64F(copTrajectoryOrder + 1, 1);
      tempTrajectory.setSeptic(0.0, 0.05, 0.10, 0.15, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0);
      tempTrajectory.getCoefficientVector(yAxisTrajectoryCoefficients);
      helper.generateSupportPolygonConstraint(xAxisConstraintCoefficientToSet, yAxisConstraintCoefficientToSet, biasMatrixToSet, xAxisTrajectoryCoefficients,
                                              yAxisTrajectoryCoefficients, supportPolygon, nodeTimes, copTrajectoryOrder);
      PrintTools.debug(xAxisConstraintCoefficientToSet.toString());
      PrintTools.debug(yAxisConstraintCoefficientToSet.toString());
      PrintTools.debug(biasMatrixToSet.toString());
   }
   
   @Test
   public void testObjectiveGeneration()
   {
      CollinearForcePlannerOptimizationControlModuleHelper helper = new CollinearForcePlannerOptimizationControlModuleHelper();
      DenseMatrix64F H = new DenseMatrix64F(0, 1);
      DenseMatrix64F f = new DenseMatrix64F(0, 1);
      DenseMatrix64F coefficients = new DenseMatrix64F(8, 1);
      //coefficients.setData(new double[] {0.0, 0.0, 1.0 / 2.0 , 1.0 / (3.0 * 2.0), 1.0 / (4.0 * 3.0), 1.0 / (5.0 * 4.0), 1.0 / (6.0 * 5.0), 1.0 / (7.0 * 6.0)});
      coefficients.setData(new double[] {1.0, 2.0, 3.0, 4.0});
      helper.generateAccelerationMinimizationObjective(H, f, coefficients, 3, 1.0);
   }
}
