package us.ihmc.manipulation.planning.exploringSpatial;

import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ExploringDefinitionForGeneral extends ExploringDefinition
{
   private double trajectoryTime = 0.0;

   // TODO.
   private final static int indexOfGoalManifold = 0;

   private double initialDistanceToGoalManifold = 0.0;

   private RigidBodyTransform closestTransformOnGoalManifold = new RigidBodyTransform();

   private final double positionWeight = 1.0;
   private final double orientationWeight = 0.0;

   private final RigidBody exploringHand;

   public ExploringDefinitionForGeneral(List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                                        List<RigidBodyExplorationConfigurationCommand> explorationConfigurations, List<ReachingManifoldCommand> manifolds)
   {
      super(endEffectorTrajectories, explorationConfigurations, manifolds);

      for (int i = 0; i < endEffectorTrajectories.size(); i++)
      {
         WaypointBasedTrajectoryCommand trajectoryCommand = endEffectorTrajectories.get(i);
         if (trajectoryTime < trajectoryCommand.getLastWaypointTime())
            trajectoryTime = trajectoryCommand.getLastWaypointTime();
      }

      ReachingManifoldCommand reachingManifoldCommand = goalManifolds.get(indexOfGoalManifold);
      exploringHand = reachingManifoldCommand.getRigidBody();
      RigidBodyTransform handTransform = getExploringRigidBodyTransformToWorld(new SpatialNode(createDefaultSpatialData()), exploringHand);

      PrintTools.info("handTransform");
      System.out.println(handTransform);

      updateClosestTransformOnGoalManifold(handTransform);
      initialDistanceToGoalManifold = getDistanceToGoalManifold(handTransform);

      PrintTools.info("trajectoryTime is " + trajectoryTime);
      PrintTools.info("intial distance is " + initialDistanceToGoalManifold);
   }

   public double getExploringProgress(SpatialNode node)
   {
      RigidBodyTransform handTransform = getExploringRigidBodyTransformToWorld(node, exploringHand);

      updateClosestTransformOnGoalManifold(handTransform);
      double distance = getDistanceToGoalManifold(handTransform);
      return 1.0 - distance / initialDistanceToGoalManifold;
   }

   private double getDistanceToGoalManifold(RigidBodyTransform handTransform)
   {
      return ReachingManifoldTools.getDistance(handTransform, closestTransformOnGoalManifold, positionWeight, orientationWeight);
   }

   private void updateClosestTransformOnGoalManifold(RigidBodyTransform rigidBodyTransform)
   {
      ReachingManifoldCommand reachingManifoldCommand = goalManifolds.get(indexOfGoalManifold);
      ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(reachingManifoldCommand, rigidBodyTransform, closestTransformOnGoalManifold, positionWeight, orientationWeight);
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   private RigidBodyTransform getExploringRigidBodyTransformToWorld(SpatialNode node, RigidBody rigidBody)
   {
      double timeInTrajectory = node.getTime();

      RigidBodyTransform poseToAppend = node.getSpatialData(rigidBody);

      ExploringRigidBody exploringRigidBody = null;
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
         if (allExploringRigidBodies.get(i).getRigidBody() == rigidBody)
            exploringRigidBody = allExploringRigidBodies.get(i);

      if (exploringRigidBody != null)
         return exploringRigidBody.getRigidBodyTransform(timeInTrajectory, poseToAppend);
      else
         return null;
   }

   public SpatialNode createSpatialNodeOnGoalManifold(SpatialNode saturatedNode, double timeDiff)
   {
      double nodeTime = saturatedNode.getTime() + timeDiff;
      
      MathTools.clamp(nodeTime, 0.0, getTrajectoryTime());
      
      SpatialData spatialData = new SpatialData(saturatedNode.getSpatialData());

      RigidBodyTransform expectedHandTransform = getExploringRigidBodyTransformToWorld(new SpatialNode(nodeTime, spatialData), exploringHand);

      updateClosestTransformOnGoalManifold(expectedHandTransform);

      RigidBodyTransform transformHandtoManifold = new RigidBodyTransform(closestTransformOnGoalManifold);
      transformHandtoManifold.preMultiplyInvertOther(expectedHandTransform);

      spatialData.replaceSpatialData(exploringHand.getName(), transformHandtoManifold);

      SpatialNode node = new SpatialNode(nodeTime, spatialData);

      RigidBodyTransform extimatedHandTransform = getExploringRigidBodyTransformToWorld(node, exploringHand);

      updateClosestTransformOnGoalManifold(extimatedHandTransform);
      double estimatedDistance = getDistanceToGoalManifold(extimatedHandTransform);

      return node;
   }
}
