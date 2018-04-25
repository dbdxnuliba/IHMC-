package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;

public class ExploringConfigurationSpace
{
   private final ConfigurationSpaceName configurationSpaceName;

   private TDoubleArrayList configuration = new TDoubleArrayList();

   private double lowerLimit;
   private double upperLimit;

   public ExploringConfigurationSpace(ConfigurationSpaceName configurationSpaceName, double lowerLimit, double upperLimit)
   {
      this.configurationSpaceName = configurationSpaceName;

      if (configurationSpaceName == ConfigurationSpaceName.SE3)
      {
         this.configuration.addAll(new double[] {0, 0, 0});
      }
      else
      {
         this.configuration.add(0.0);
      }

      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;
   }

   public ConfigurationSpaceName getConfigurationSpaceName()
   {
      return configurationSpaceName;
   }

   public double[] getConfigurations()
   {
      return configuration.toArray();
   }

   public void interpolate(ExploringConfigurationSpace one, ExploringConfigurationSpace two, double alpha)
   {
      if (configurationSpaceName == one.configurationSpaceName && configurationSpaceName == two.configurationSpaceName)
      {
         for (int i = 0; i < configuration.size(); i++)
         {
            double double1 = one.configuration.get(i);
            double double2 = two.configuration.get(i);
            double doubleInterpolate = double1 + (double2 - double1) * alpha;

            configuration.replace(i, doubleInterpolate);
         }
      }
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
         double randomConfiguration = ConfigurationSpaceName.random.nextDouble() * (upperLimit - lowerLimit) + lowerLimit;
         configuration.replace(i, randomConfiguration);
      }
   }

   public RigidBodyTransform getRandomLocalRigidBodyTransform()
   {
      updateRandomConfiguration();

      double[] configuration = new double[this.configuration.size()];

      for (int i = 0; i < configuration.length; i++)
      {
         configuration[i] = this.configuration.get(i);
      }

      return configurationSpaceName.getLocalRigidBodyTransform(configuration);
   }

   public int getExploringDimension()
   {
      if (configurationSpaceName == ConfigurationSpaceName.SE3)
         return 3;
      else
         return 1;
   }
}
