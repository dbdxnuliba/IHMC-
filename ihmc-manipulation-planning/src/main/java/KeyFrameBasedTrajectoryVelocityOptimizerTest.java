import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.log.LogTools;
import us.ihmc.manipulation.planning.gradientDescent.GradientDescentModule;
import us.ihmc.manipulation.planning.gradientDescent.SingleQueryFunction;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class KeyFrameBasedTrajectoryVelocityOptimizerTest
{
   private final TDoubleArrayList positions = new TDoubleArrayList();
   private final TDoubleArrayList times = new TDoubleArrayList();
   private static final double velocitOptimizerDT = 0.001;

   private enum OptimizationType
   {
      Velocity, Acceleration, Jerk, KineticEnergy
   }

   public KeyFrameBasedTrajectoryVelocityOptimizerTest()
   {
      defineTrajectoryPoints();
      TDoubleArrayList velocities = calculateInitialVelocitiesExceptFirstAndFinal(times, positions);

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.addAll(velocities);

      for (int i = 0; i < initial.size(); i++)
         System.out.println("# initial input is " + initial.get(i));

      TDoubleArrayList initialVelocities = new TDoubleArrayList();
      initialVelocities.add(0.0);
      initialVelocities.addAll(initial);
      initialVelocities.add(0.0);
      List<Trajectory> originalTrajectories = calculateTrajectories(times, positions, initialVelocities);
      saveJointPositionAndVelocity("initialTrajectory", originalTrajectories, velocitOptimizerDT);
      
      runOptimizer(OptimizationType.Velocity, initial);
      runOptimizer(OptimizationType.Acceleration, initial);
      runOptimizer(OptimizationType.Jerk, initial);
      runOptimizer(OptimizationType.KineticEnergy, initial);
   }

   private void runOptimizer(OptimizationType optimizationType, TDoubleArrayList initial)
   {
      SingleQueryFunction function = new EvaluationFunction(optimizationType);
      GradientDescentModule optimizer = new GradientDescentModule(function, initial);
      optimizer.setMaximumIterations(100);
      
      LogTools.info("initial " + optimizationType.toString() + function.getQuery(initial));

      System.out.println("iteration is " + optimizer.run());
      TDoubleArrayList optimalSolution = optimizer.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      TDoubleArrayList finalVelocities = new TDoubleArrayList();
      finalVelocities.add(0.0);
      finalVelocities.addAll(optimalSolution);
      finalVelocities.add(0.0);
      List<Trajectory> optimalTrajectories = calculateTrajectories(times, positions, finalVelocities);
      saveJointPositionAndVelocity("optimalTrajectory_" + optimizationType.toString(), optimalTrajectories, velocitOptimizerDT);

      System.out.println("optimal " + optimizationType.toString() + optimizer.getOptimalQuery());
   }

   private class EvaluationFunction implements SingleQueryFunction
   {
      private OptimizationType optimizationType;

      private EvaluationFunction(OptimizationType optimizationType)
      {
         this.optimizationType = optimizationType;
      }

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         int numberOfTicks = (int) (times.get(times.size() - 1) / velocitOptimizerDT);
         double time = 0.0;
         double kineticEnergy = 0.0;
         for (int i = 1; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(time);
            TDoubleArrayList velocities = new TDoubleArrayList();
            velocities.add(0.0);
            velocities.addAll(values);
            velocities.add(0.0);
            List<Trajectory> trajectories = calculateTrajectories(times, positions, velocities);
            Trajectory trajectory = trajectories.get(indexOfTrajectory);
            trajectory.compute(time);

            double previousTime = time - velocitOptimizerDT;
            int indexOfTrajectoryForPreviousTick = findTrajectoryIndex(previousTime);
            Trajectory trajectoryForPreviousTick = trajectories.get(indexOfTrajectoryForPreviousTick);
            trajectoryForPreviousTick.compute(previousTime);

            switch (optimizationType)
            {
            case Velocity:
               kineticEnergy += Math.abs(trajectory.getVelocity()) * velocitOptimizerDT;
               break;
            case Acceleration:
               kineticEnergy += Math.abs(trajectory.getAcceleration()) * velocitOptimizerDT;
               break;
            case Jerk:
               kineticEnergy += Math.abs(trajectory.getAcceleration() - trajectoryForPreviousTick.getAcceleration()) / velocitOptimizerDT;
               break;
            case KineticEnergy:
               kineticEnergy += trajectory.getVelocity() * trajectory.getVelocity() * velocitOptimizerDT;
               break;
            }
            time += velocitOptimizerDT;
         }
         return kineticEnergy;
      }
   }

   private void defineTrajectoryPoints()
   {
      positions.add(1.0);
      positions.add(2.0);
      positions.add(4.0);
      positions.add(3.0);
      positions.add(1.0);

      times.add(0.0);
      times.add(1.0);
      times.add(2.0);
      times.add(3.5);
      times.add(5.0);
   }

   private int findTrajectoryIndex(double time)
   {
      int indexOfTrajectory = 0;
      for (int j = times.size() - 1; j > 0; j--)
      {
         if (times.get(j) < time)
         {
            indexOfTrajectory = j;
            break;
         }
      }
      return indexOfTrajectory;
   }

   private List<Trajectory> calculateTrajectories(TDoubleArrayList times, TDoubleArrayList positions, TDoubleArrayList velocities)
   {
      List<Trajectory> trajectories = new ArrayList<Trajectory>();

      for (int j = 0; j < times.size() - 1; j++)
      {
         Trajectory cubic = new Trajectory(4);
         cubic.setCubic(times.get(j), times.get(j + 1), positions.get(j), velocities.get(j), positions.get(j + 1), velocities.get(j + 1));
         trajectories.add(cubic);
      }

      return trajectories;
   }

   private TDoubleArrayList calculateInitialVelocitiesExceptFirstAndFinal(TDoubleArrayList times, TDoubleArrayList positions)
   {
      TDoubleArrayList velocities = new TDoubleArrayList();
      for (int i = 1; i < times.size() - 1; i++)
         velocities.add((positions.get(i + 1) - positions.get(i)) / (times.get(i + 1) - times.get(i)));

      return velocities;
   }

   private void saveJointPositionAndVelocity(String namePrefix, List<Trajectory> trajectories, double timeTick)
   {
      int numberOfTicks = (int) (times.get(times.size() - 1) / timeTick);
      FileWriter positionFW;
      try
      {
         positionFW = new FileWriter(new File(namePrefix + "_Pos_Vel_Acc.csv"));

         positionFW.write(String.format("time\t_position\t_velocity\t_acceleration"));
         positionFW.write(System.lineSeparator());

         double time = 0.0;
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(time);

            positionFW.write(String.format("%.4f (%d)", time, indexOfTrajectory));
            Trajectory trajectory = trajectories.get(indexOfTrajectory);
            trajectory.compute(time);

            positionFW.write(String.format("\t%.4f\t%.4f\t%.4f", trajectory.getPosition(), trajectory.getVelocity(), trajectory.getAcceleration()));

            positionFW.write(System.lineSeparator());
            time += timeTick;
         }

         positionFW.close();
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
      }

      LogTools.info("done");
   }

   public static void main(String[] args)
   {
      KeyFrameBasedTrajectoryVelocityOptimizerTest test = new KeyFrameBasedTrajectoryVelocityOptimizerTest();

      LogTools.info("done");
   }
}
