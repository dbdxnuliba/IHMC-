package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;

public class ExploringConfigurationSpace
{
   private final Random random = new Random(1);

   private final ConfigurationSpaceName configurationSpaceName;

   private TDoubleArrayList configuration = new TDoubleArrayList();

   private double lowerLimit;
   private double upperLimit;

   public ExploringConfigurationSpace(ConfigurationSpaceName configurationSpaceName, double lowerLimit, double upperLimit)
   {
      this.configurationSpaceName = configurationSpaceName;

      if (configurationSpaceName == ConfigurationSpaceName.SE3)
         this.configuration.addAll(new double[] {0, 0, 0});
      else
         this.configuration.add(0.0);

      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;
   }

   public ConfigurationSpaceName getConfigurationSpaceName()
   {
      return configurationSpaceName;
   }

   public double getConfiguration()
   {
      // TODO : implement for SE3.
      return configuration.get(0);
   }

   public void setConfiguration(double configuration)
   {
      if (configurationSpaceName == ConfigurationSpaceName.SE3)
         System.err.println("SE3 configuration has Quaternion value");
      else
         this.configuration.replace(0, configuration);
   }

   public void setUpperLimit(double value)
   {
      upperLimit = value;
   }

   public void setLowerLimit(double value)
   {
      lowerLimit = value;
   }

   private void updateRandomConfiguration()
   {
      for (int i = 0; i < configuration.size(); i++)
      {
         double randomConfiguration = random.nextDouble() * (upperLimit - lowerLimit) + lowerLimit;
         configuration.replace(i, randomConfiguration);
      }
   }

   public RigidBodyTransform getRandomLocalRigidBodyTransform()
   {
      updateRandomConfiguration();

      double[] configuration = new double[this.configuration.size()];

      for (int i = 0; i < configuration.length; i++)
         configuration[i] = this.configuration.get(i);

      return configurationSpaceName.getLocalRigidBodyTransform(configuration);
   }
}
