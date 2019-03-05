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

   public KeyFrameBasedTrajectoryVelocityOptimizerTest()
   {
      defineTrajectoryPoints();
      TDoubleArrayList velocities = calculateInitialVelocitiesExceptFirstAndFinal(times, positions);

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.addAll(velocities);

      for (int i = 0; i < initial.size(); i++)
         System.out.println("initial is " + initial.get(i));

      SingleQueryFunction function = new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            int numberOfTicks = (int) (times.get(times.size() - 1) / velocitOptimizerDT);
            double time = 0.0;
            double kineticEnergy = 0.0;
            for (int i = 0; i < numberOfTicks; i++)
            {
               int indexOfTrajectory = findTrajectoryIndex(time);
               TDoubleArrayList velocities = new TDoubleArrayList();
               velocities.add(0.0);
               velocities.addAll(values);
               velocities.add(0.0);
               List<Trajectory> trajectories = calculateTrajectories(times, positions, velocities);
               Trajectory trajectory = trajectories.get(indexOfTrajectory);
               trajectory.compute(time);

               // TODO : compare which way is better between square of velocity and absolute value of velocity.
               //kineticEnergy += trajectory.getVelocity() * trajectory.getVelocity() * velocitOptimizerDT;
               //kineticEnergy += Math.abs(trajectory.getVelocity()) * velocitOptimizerDT;
               kineticEnergy += Math.abs(trajectory.getAcceleration()) * velocitOptimizerDT;
               time += velocitOptimizerDT;
            }
            return kineticEnergy;
         }
      };
      GradientDescentModule solver = new GradientDescentModule(function, initial);
      solver.setMaximumIterations(100);

      LogTools.info("initial energy " + function.getQuery(initial));
      TDoubleArrayList initialVelocities = new TDoubleArrayList();
      initialVelocities.add(0.0);
      initialVelocities.addAll(initial);
      initialVelocities.add(0.0);
      List<Trajectory> originalTrajectories = calculateTrajectories(times, positions, initialVelocities);
      saveJointPositionAndVelocity("initial", originalTrajectories, velocitOptimizerDT);

      System.out.println("iteration is " + solver.run());
      TDoubleArrayList optimalSolution = solver.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      TDoubleArrayList finalVelocities = new TDoubleArrayList();
      finalVelocities.add(0.0);
      finalVelocities.addAll(optimalSolution);
      finalVelocities.add(0.0);
      List<Trajectory> optimalTrajectories = calculateTrajectories(times, positions, finalVelocities);
      saveJointPositionAndVelocity("optimal", optimalTrajectories, velocitOptimizerDT);

      System.out.println("optimal query is " + solver.getOptimalQuery());
   }

   private void defineTrajectoryPoints()
   {
      positions.add(1.0);
      positions.add(2.0);
      positions.add(3.0);
      positions.add(4.0);
      positions.add(5.0);

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

   public static void main(String[] args)
   {
      KeyFrameBasedTrajectoryVelocityOptimizerTest test = new KeyFrameBasedTrajectoryVelocityOptimizerTest();

      LogTools.info("done");
   }

   private void saveJointPositionAndVelocity(String namePrefix, List<Trajectory> trajectories, double timeTick)
   {
      int numberOfTicks = (int) (times.get(times.size() - 1) / timeTick);
      FileWriter positionFW;
      try
      {
         positionFW = new FileWriter(new File(namePrefix + "_position_velocity.csv"));

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
}
