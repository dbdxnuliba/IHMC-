package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.robotics.geometry.AngleTools;

/**
 * this is data structure for plotting of the {@link SpatialNode}
 * @author shadylady
 *
 */
public class SpatialData
{
   private final List<ExploringRigidBody> exploringRigidBodies;
   private final List<Pose3D> rigidBodySpatials;

   private final List<ExploringConfigurationSpace> exploringConfigurationSpaces;

   public SpatialData()
   {
      exploringRigidBodies = new ArrayList<ExploringRigidBody>();
      rigidBodySpatials = new ArrayList<Pose3D>();

      exploringConfigurationSpaces = new ArrayList<ExploringConfigurationSpace>();
   }

   public SpatialData(SpatialData other)
   {
      this();
      exploringRigidBodies.addAll(other.exploringRigidBodies);
      rigidBodySpatials.addAll(other.rigidBodySpatials);
      exploringConfigurationSpaces.addAll(other.exploringConfigurationSpaces);
   }

   public void appendSpatial(ExploringRigidBody exploringRigidBody, List<ExploringConfigurationSpace> exploringConfigurations, RigidBodyTransform pose)
   {
      this.exploringRigidBodies.add(exploringRigidBody);
      this.rigidBodySpatials.add(new Pose3D(pose));
      this.exploringConfigurationSpaces.addAll(exploringConfigurations);
   }

   public void interpolate(SpatialData dataOne, SpatialData dataTwo, double alpha)
   {
      for (int i = 0; i < rigidBodySpatials.size(); i++)
         rigidBodySpatials.get(i).interpolate(dataOne.getRigidBodySpatials().get(i), dataTwo.getRigidBodySpatials().get(i), alpha);

      for (int i = 0; i < exploringConfigurationSpaces.size(); i++)
         exploringConfigurationSpaces.get(i).interpolate(dataOne.exploringConfigurationSpaces.get(i), dataTwo.exploringConfigurationSpaces.get(i), alpha);
   }

   public double getPositionDistance(SpatialData other)
   {
      double distance = 0.0;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         if (!getRigidBodyName(i).equals(other.getRigidBodyName(i)))
            PrintTools.warn("other spatial data has different order");

         distance = distance + rigidBodySpatials.get(i).getPositionDistance(other.getRigidBodySpatials().get(i));
      }

      return distance;
   }

   public double getOrientationDistance(SpatialData other)
   {
      double distance = 0.0;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double orientationDistance;

         if (rigidBodySpatials.get(i).getOrientation().equals(other.getRigidBodySpatials().get(i).getOrientation()))
            orientationDistance = 0.0;
         else
            orientationDistance = rigidBodySpatials.get(i).getOrientationDistance(other.getRigidBodySpatials().get(i));

         orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
         orientationDistance = Math.abs(orientationDistance);

         distance = distance + orientationDistance;
      }

      return distance;
   }

   public double getMaximumPositionDistance(SpatialData other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double positionDistance = rigidBodySpatials.get(i).getPositionDistance(other.getRigidBodySpatials().get(i));

         if (distance < positionDistance)
            distance = positionDistance;
      }

      return distance;
   }

   public double getMaximumOrientationDistance(SpatialData other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double orientationDistance = rigidBodySpatials.get(i).getOrientationDistance(other.getRigidBodySpatials().get(i));
         orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
         orientationDistance = Math.abs(orientationDistance);
         if (distance < orientationDistance)
            distance = orientationDistance;
      }

      return distance;
   }

   public Pose3D getTestFrame(List<ReachingManifoldCommand> manifolds)
   {
      for (int j = 0; j < manifolds.size(); j++)
      {
         for (int i = 0; i < rigidBodySpatials.size(); i++)
         {
            if (getRigidBodyName(i).equals(manifolds.get(j).getRigidBody().getName()))
            {
               ReachingManifoldCommand manifold = manifolds.get(j);
               Pose3D currentSpatial = rigidBodySpatials.get(i);

               return manifold.computeClosestPoseOnManifold(currentSpatial);
            }
         }
      }
      return null;
   }

   public String getRigidBodyName(int i)
   {
      return exploringRigidBodies.get(i).getRigidBody().getName();
   }

   public List<Pose3D> getRigidBodySpatials()
   {
      return rigidBodySpatials;
   }

   public int getNumberOfExploringRigidBodies()
   {
      return exploringRigidBodies.size();
   }
   
   public int getExploringDimension()
   {
      int exploringDimension = 0;

      for (int i = 0; i < exploringConfigurationSpaces.size(); i++)
         exploringDimension = exploringDimension + exploringConfigurationSpaces.get(i).getExploringDimension();

      return exploringDimension;
   }

   public List<String> getExploringConfigurationNames()
   {
      List<String> names = new ArrayList<String>();

      for (int i = 0; i < exploringRigidBodies.size(); i++)
      {
         ExploringRigidBody exploringRigidBody = this.exploringRigidBodies.get(i);
         for (int j = 0; j < exploringRigidBody.getExploringConfigurationSpaces().size(); j++)
         {
            ExploringConfigurationSpace exploringConfigurationSpace = exploringRigidBody.getExploringConfigurationSpaces().get(j);
            ConfigurationSpaceName configurationSpaceName = exploringConfigurationSpace.getConfigurationSpaceName();

            if (configurationSpaceName == ConfigurationSpaceName.SE3)
            {
               names.add(exploringRigidBody.getRigidBody().getName() + "_" + ConfigurationSpaceName.SE3.name() + "_Roll");
               names.add(exploringRigidBody.getRigidBody().getName() + "_" + ConfigurationSpaceName.SE3.name() + "_Pitch");
               names.add(exploringRigidBody.getRigidBody().getName() + "_" + ConfigurationSpaceName.SE3.name() + "_Yaw");
            }
            else
               names.add(exploringRigidBody.getRigidBody().getName() + "_" + configurationSpaceName.name());
         }

      }

      return names;
   }

   public TDoubleArrayList getExploringConfigurations()
   {
      TDoubleArrayList configurations = new TDoubleArrayList();

      for (int i = 0; i < exploringConfigurationSpaces.size(); i++)
      {
         ExploringConfigurationSpace exploringConfigurationSpace = exploringConfigurationSpaces.get(i);
         configurations.addAll(exploringConfigurationSpace.getConfigurations());
      }

      return configurations;
   }
}
