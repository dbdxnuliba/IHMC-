package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RandomSpatialGenerator
{
   private final Random random = new Random(1);

   private final RigidBody rigidBody;

   private final ArrayList<ConfigurationSpaceName> degreesOfFreedomToExplore = new ArrayList<>();

   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

   public RandomSpatialGenerator(RigidBody rigidBody, RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      this.rigidBody = rigidBody;
   }

   public boolean equals(String rigidBodyName)
   {
      return rigidBodyName.equals(rigidBody.getName());
   }

   public Pose3D getRandomSpatial()
   {
      Pose3D randomSpatial = new Pose3D();

      for (int i = 0; i < degreesOfFreedomToExplore.size(); i++)
      {
         double randomConfiguration = random.nextDouble() * (explorationRangeUpperLimits.get(i) - explorationRangeLowerLimits.get(i))
               + explorationRangeLowerLimits.get(i);
         switch (degreesOfFreedomToExplore.get(i))
         {
         case X:
         case Y:
         case Z:
         case ROLL:
         case PITCH:
         case YAW:
            randomSpatial.appendTransform(degreesOfFreedomToExplore.get(i).getLocalRigidBodyTransform(randomConfiguration));
            break;
         case SE3:
            randomSpatial.appendTransform(degreesOfFreedomToExplore.get(i).getLocalRigidBodyTransform(random.nextDouble(), random.nextDouble(),
                                                                                                      random.nextDouble()));
            break;
         }
      }
      return null;
   }

   private double getRandomConfiguration(int i)
   {
      double random = this.random.nextDouble();
      return random * (explorationRangeUpperLimits.get(i) - explorationRangeLowerLimits.get(i)) + explorationRangeLowerLimits.get(i);
   }
}
