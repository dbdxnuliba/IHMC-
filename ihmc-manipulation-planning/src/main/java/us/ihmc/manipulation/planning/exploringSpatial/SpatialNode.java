package us.ihmc.manipulation.planning.exploringSpatial;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * SpatialNode includes SpatialData and time.
 * This follows the characteristics of common node.
 * So, this has parent node and its validity.
 * In addition, this has joint configuration which is represented by KinematicsToolboxOutputStatus.
 */
public class SpatialNode
{
   private double time;
   private SpatialData spatialData;

   private SpatialNode parent = null;

   private boolean validity = true;
   private KinematicsToolboxOutputStatus configuration = null;

   public SpatialNode(SpatialData spatialData)
   {
      this(0.0, spatialData);
   }

   public SpatialNode(double time, SpatialData spatialData)
   {
      this.time = time;
      this.spatialData = new SpatialData(spatialData);
   }

   public SpatialNode(SpatialNode other)
   {
      time = other.time;
      spatialData = new SpatialData(other.spatialData);

      if (other.parent != null)
         parent = new SpatialNode(other.parent);
      //parent = other.parent;
      else
         parent = null;

      validity = other.validity;
      if (other.configuration != null)
         configuration = new KinematicsToolboxOutputStatus(other.configuration);
   }

   public double getTimeGap(SpatialNode other)
   {
      if (getTime() > other.getTime())
         return Double.MAX_VALUE;
      else
         return other.getTime() - getTime();
   }

   private double getPositionDistance(SpatialNode other)
   {
      return spatialData.getPositionDistance(other.getSpatialData());
   }

   private double getOrientationDistance(SpatialNode other)
   {
      return spatialData.getOrientationDistance(other.getSpatialData());
   }

   public double computeDistanceWithinMaxDistance(double timeWeight, double positionWeight, double orientationWeight, SpatialNode other, double maxTimeInterval,
                                                  double maxPositionDistance, double maxOrientationDistance)
   {
      double timeDistance = timeWeight * getTimeGap(other);
      double positionDistance = positionWeight * getPositionDistance(other);
      double orientationDistance = orientationWeight * getOrientationDistance(other);

      double greatestPositionDistance = spatialData.getMaximumPositionDistance(other.getSpatialData());
      double greatestOrientationDistance = spatialData.getMaximumOrientationDistance(other.getSpatialData());

      if (greatestPositionDistance / getTimeGap(other) > maxPositionDistance / maxTimeInterval)
         return Double.MAX_VALUE;
      if (greatestOrientationDistance / getTimeGap(other) > maxOrientationDistance / maxTimeInterval)
         return Double.MAX_VALUE;

      double distance = timeDistance + positionDistance + orientationDistance;

      return distance;
   }

   public void interpolate(SpatialNode nodeOne, SpatialNode nodeTwo, double alpha)
   {
      time = EuclidCoreTools.interpolate(nodeOne.time, nodeTwo.time, alpha);
      spatialData.interpolate(nodeOne.getSpatialData(), nodeTwo.getSpatialData(), alpha);
   }

   public SpatialData getSpatialData()
   {
      return spatialData;
   }

   public void setParent(SpatialNode parent)
   {
      this.parent = parent;
   }

   public void clearParent()
   {
      parent = null;
   }

   public SpatialNode getParent()
   {
      return parent;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public double getTime()
   {
      return time;
   }

   public void setConfiguration(KinematicsToolboxOutputStatus configuration)
   {
      this.configuration = new KinematicsToolboxOutputStatus(configuration);
   }

   public KinematicsToolboxOutputStatus getConfiguration()
   {
      return configuration;
   }

   public void setValidity(boolean validity)
   {
      this.validity = validity;
   }

   public boolean isValid()
   {
      return validity;
   }

   public RigidBodyTransform getSpatialData(int index)
   {
      return spatialData.getRigidBodySpatials().get(index);
   }

   public RigidBodyTransform getSpatialData(RigidBody rigidBody)
   {
      for (int i = 0; i < spatialData.getNumberOfExploringRigidBodies(); i++)
      {
         if (spatialData.getRigidBodyName(i).equals(rigidBody.getName()))
            return getSpatialData(i);
      }
      return null;
   }
}