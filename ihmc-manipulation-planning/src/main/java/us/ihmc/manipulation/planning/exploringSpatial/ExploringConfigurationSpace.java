package us.ihmc.manipulation.planning.exploringSpatial;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;

public class ExploringConfigurationSpace
{
   private final ConfigurationSpaceName configurationSpaceName;

   private TDoubleArrayList configuration = new TDoubleArrayList();

   private double lowerLimit;
   private double upperLimit;

   public ExploringConfigurationSpace(ExploringConfigurationSpace other)
   {
      this(other.configurationSpaceName, other.lowerLimit, other.upperLimit);
      for (int i = 0; i < other.configuration.size(); i++)
         configuration.set(i, other.configuration.get(i));
   }

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

      // TODO : default set is required.
      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;
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
}
