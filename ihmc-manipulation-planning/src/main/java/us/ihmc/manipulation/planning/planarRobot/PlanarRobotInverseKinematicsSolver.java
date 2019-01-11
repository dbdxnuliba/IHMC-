package us.ihmc.manipulation.planning.planarRobot;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;

public class PlanarRobotInverseKinematicsSolver implements InverseKinematicsInterface
{
   private static final int maximumNumberOfIteration = 20;
   private static final double perturbedJoint = 0.002;
   private static final double minimalDampedCoefficient = 0.0001;
   private static final double residualDampedCoefficient = 1;

   private final PlanarRobot robot;

   private final int taskDimension;
   private final int jointDimension;

   private final TDoubleArrayList desiredTaskSpace;
   private final TDoubleArrayList currentJointSpace;
   private final TDoubleArrayList currentTaskSpace;

   private final DenseMatrix64F taskErrorSpace;

   private final TDoubleArrayList perturbedJointSpace;
   private final TDoubleArrayList perturbedTaskSpace;

   private final DenseMatrix64F jacobianMatrix;
   private final DenseMatrix64F jacobianTransposeMatrix;

   public PlanarRobotInverseKinematicsSolver(PlanarRobot robot)
   {
      this.robot = new PlanarRobot(robot);

      taskDimension = robot.getTaskDimension();
      jointDimension = robot.getJointDimension();

      desiredTaskSpace = new TDoubleArrayList();
      currentJointSpace = new TDoubleArrayList();
      currentTaskSpace = new TDoubleArrayList();
      taskErrorSpace = new DenseMatrix64F(taskDimension, 1);
      perturbedJointSpace = new TDoubleArrayList();
      perturbedTaskSpace = new TDoubleArrayList();

      jacobianMatrix = new DenseMatrix64F(taskDimension, jointDimension);
      jacobianTransposeMatrix = new DenseMatrix64F(jointDimension, taskDimension);
   }

   @Override
   public boolean solve()
   {
      for (int i = 0; i < maximumNumberOfIteration; i++)
      {
         // get current task space.
         currentTaskSpace.clear();
         robot.setJointConfiguration(currentJointSpace);
         for (int j = 0; j < taskDimension; j++)
            currentTaskSpace.addAll(robot.getTaskConfiguration());

         // get task error space.
         // add break statement if error is small enough not to need finding new solution.
         for (int j = 0; j < taskDimension; j++)
            taskErrorSpace.set(j, 0, desiredTaskSpace.get(j) - currentTaskSpace.get(j));

         // get jacobian matrix.
         for (int j = 0; j < jointDimension; j++)
         {
            perturbedJointSpace.clear();
            perturbedTaskSpace.clear();
            perturbedJointSpace.addAll(currentJointSpace);
            perturbedJointSpace.replace(j, currentJointSpace.get(j) + perturbedJoint);
            robot.setJointConfiguration(perturbedJointSpace);
            perturbedTaskSpace.addAll(robot.getTaskConfiguration());
            for (int k = 0; k < taskDimension; k++)
            {
               double deltaTask = perturbedTaskSpace.get(k) - currentTaskSpace.get(k);
               double deltaJoint = perturbedJoint;
               double value = deltaTask / deltaJoint;

               jacobianMatrix.set(k, j, value);
            }
         }

         // get current g.
         for (int j = 0; j < jointDimension; j++)
            for (int k = 0; k < taskDimension; k++)
               jacobianTransposeMatrix.set(j, k, jacobianMatrix.get(k, j));

         DenseMatrix64F gMatrix = new DenseMatrix64F(jointDimension, 1);
         NativeCommonOps.mult(jacobianTransposeMatrix, taskErrorSpace, gMatrix);

         // get hessian.
         DenseMatrix64F hessian = new DenseMatrix64F(jointDimension, jointDimension);
         NativeCommonOps.mult(jacobianTransposeMatrix, jacobianMatrix, hessian);

         // add damped
         double errorSquare = 0;
         for (int j = 0; j < taskDimension; j++)
            errorSquare = errorSquare + taskErrorSpace.get(j, 0) * taskErrorSpace.get(j, 0);
         double dampedCoefficient = errorSquare * residualDampedCoefficient + minimalDampedCoefficient;
         DenseMatrix64F dampedHessian = new DenseMatrix64F(hessian);
         for (int j = 0; j < jointDimension; j++)
            dampedHessian.add(j, j, dampedCoefficient);

         LogTools.info("" + i + " errorSquare " + errorSquare);

         // get inverse of hessian.
         DenseMatrix64F inverseHessian = new DenseMatrix64F(jointDimension, jointDimension);
         NativeCommonOps.invert(dampedHessian, inverseHessian);

         // get required diff of joint space.
         DenseMatrix64F deltaJointSpace = new DenseMatrix64F(jointDimension, 1);
         NativeCommonOps.mult(inverseHessian, gMatrix, deltaJointSpace);

         // append on current joint space to get new joint space.
         for (int j = 0; j < jointDimension; j++)
         {
            currentJointSpace.set(j, currentJointSpace.get(j) + deltaJointSpace.get(j));
         }
      }

      return true;
   }

   @Override
   public void setInitialConfiguration(TDoubleArrayList jointConfiguration)
   {
      currentJointSpace.clear();
      currentJointSpace.addAll(jointConfiguration);
   }

   @Override
   public void setDesiredTaskConfiguration(TDoubleArrayList taskConfiguration)
   {
      if (robot.getTaskDimension() != taskConfiguration.size())
         LogTools.error("task dimension is differnt (robot : " + robot.getTaskDimension() + ", desired : " + taskConfiguration.size() + ").");
      desiredTaskSpace.clear();
      desiredTaskSpace.addAll(taskConfiguration);
   }

   @Override
   public TDoubleArrayList getSolution()
   {
      return currentJointSpace;
   }

}
