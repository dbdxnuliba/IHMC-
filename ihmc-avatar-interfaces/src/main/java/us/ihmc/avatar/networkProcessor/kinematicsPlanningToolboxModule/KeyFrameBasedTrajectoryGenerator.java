package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.fest.swing.util.Pair;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class KeyFrameBasedTrajectoryGenerator
{
   /*
    * keyFrames includes initial configuration. keyFrameTimes includes 0.0 (s)
    * for initial configuration.
    */
   private final List<KinematicsToolboxOutputStatus> keyFrames = new ArrayList<KinematicsToolboxOutputStatus>();
   private final TDoubleArrayList keyFrameTimes = new TDoubleArrayList();

   private final List<String> jointNames = new ArrayList<String>();
   private final Map<String, List<Trajectory>> jointNameToTrajectoriesMap = new HashMap<>();

   private final KinematicsToolboxOutputConverter converter;

   private final Map<String, Pair<Double, Double>> jointNameToVelocityBoundMap = new HashMap<>();

   private final boolean DEBUG_TRAJECTORY_PREVIEW = false;

   public KeyFrameBasedTrajectoryGenerator(DRCRobotModel drcRobotModel, List<String> jointNamesToGenerate)
   {
      converter = new KinematicsToolboxOutputConverter(drcRobotModel);
      jointNames.addAll(jointNamesToGenerate);

      for (String jointName : jointNames)
         jointNameToTrajectoriesMap.put(jointName, new ArrayList<Trajectory>());
   }

   public void compute(double searchingTimeTick)
   {
      initializeTrajectoryGenerator();
      computeOptimizedVelocity();
      computeMaximumVelocity(searchingTimeTick);

      if (DEBUG_TRAJECTORY_PREVIEW)
         saveJointPositionAndVelocity(searchingTimeTick);
   }

   public void addInitialConfiguration(KinematicsToolboxOutputStatus initialConfiguration)
   {
      if (keyFrames.size() > 0 || keyFrameTimes.size() > 0)
      {
         LogTools.warn("keyFrames should be cleared");
         clear();
      }
      keyFrames.add(initialConfiguration);
      keyFrameTimes.add(0.0);
   }

   public void addKeyFrames(List<KinematicsToolboxOutputStatus> solutionKeyFrames, TDoubleArrayList solutionKeyFrameTimes)
   {
      keyFrames.addAll(solutionKeyFrames);
      keyFrameTimes.addAll(solutionKeyFrameTimes);
   }

   private void clear()
   {
      keyFrames.clear();
      keyFrameTimes.clear();
   }

   private void initializeTrajectoryGenerator()
   {
      for (String jointName : jointNames)
      {
         TDoubleArrayList points = new TDoubleArrayList();
         for (KinematicsToolboxOutputStatus keyFrame : keyFrames)
         {
            converter.updateFullRobotModel(keyFrame);
            double point = converter.getFullRobotModel().getOneDoFJointByName(jointName).getQ();
            points.add(point);
         }

         TDoubleArrayList velocities = new TDoubleArrayList();
         velocities.add(0.0);
         for (int j = 1; j < points.size() - 1; j++)
         {
            double diff = points.get(j + 1) - points.get(j);
            double timeDiff = keyFrameTimes.get(j + 1) - keyFrameTimes.get(j);
            velocities.add(diff / timeDiff);
         }
         velocities.add(0.0);

         List<Trajectory> trajectories = new ArrayList<Trajectory>();
         for (int j = 0; j < points.size() - 1; j++)
         {
            Trajectory cubic = new Trajectory(4);
            cubic.setCubic(keyFrameTimes.get(j), keyFrameTimes.get(j + 1), points.get(j), velocities.get(j), points.get(j + 1), velocities.get(j + 1));
            trajectories.add(cubic);
         }
         jointNameToTrajectoriesMap.put(jointName, trajectories);
      }
   }

   private void computeMaximumVelocity(double searchingTimeTick)
   {
      int numberOfTicks = (int) (keyFrameTimes.get(keyFrameTimes.size() - 1) / searchingTimeTick);
      for (String jointName : jointNames)
      {
         double time = 0.0;
         TDoubleArrayList velocities = new TDoubleArrayList();
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = 0;
            for (int j = keyFrameTimes.size() - 1; j > 0; j--)
            {
               if (keyFrameTimes.get(j) < time)
               {
                  indexOfTrajectory = j;
                  break;
               }
            }
            Trajectory trajectory = jointNameToTrajectoriesMap.get(jointName).get(indexOfTrajectory);
            trajectory.compute(time);
            velocities.add(trajectory.getVelocity());

            time += searchingTimeTick;
         }
         Pair<Double, Double> minMaxVelocity = new Pair<Double, Double>(velocities.min(), velocities.max());
         jointNameToVelocityBoundMap.put(jointName, minMaxVelocity);
      }
   }

   private void saveJointPositionAndVelocity(double searchingTimeTick)
   {
      int numberOfTicks = (int) (keyFrameTimes.get(keyFrameTimes.size() - 1) / searchingTimeTick);
      FileWriter positionFW, velocityFW;
      try
      {
         positionFW = new FileWriter(new File(this.getClass().getSimpleName() + "_position.csv"));
         velocityFW = new FileWriter(new File(this.getClass().getSimpleName() + "_velocity.csv"));

         positionFW.write(String.format("time"));
         velocityFW.write(String.format("time"));
         for (String jointName : jointNames)
         {
            positionFW.write(String.format("\t%s", jointName));
            velocityFW.write(String.format("\t%s", jointName));
         }
         positionFW.write(System.lineSeparator());
         velocityFW.write(System.lineSeparator());

         double time = 0.0;
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = 0;
            for (int j = keyFrameTimes.size() - 1; j > 0; j--)
            {
               if (keyFrameTimes.get(j) < time)
               {
                  indexOfTrajectory = j;
                  break;
               }
            }

            positionFW.write(String.format("%.4f (%d)", time, indexOfTrajectory));
            velocityFW.write(String.format("%.4f (%d)", time, indexOfTrajectory));
            for (String jointName : jointNames)
            {
               Trajectory trajectory = jointNameToTrajectoriesMap.get(jointName).get(indexOfTrajectory);
               trajectory.compute(time);

               positionFW.write(String.format("\t%.4f", trajectory.getPosition()));
               velocityFW.write(String.format("\t%.4f", trajectory.getVelocity()));
            }

            positionFW.write(System.lineSeparator());
            velocityFW.write(System.lineSeparator());
            time += searchingTimeTick;
         }

         positionFW.close();
         velocityFW.close();
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
      }

      LogTools.info("done");
   }

   private void computeOptimizedVelocity()
   {

   }

   public double getJointVelocityUpperBound(String jointName)
   {
      return jointNameToVelocityBoundMap.get(jointName).ii;
   }

   public double getJointVelocityLowerBound(String jointName)
   {
      return jointNameToVelocityBoundMap.get(jointName).i;
   }
}
