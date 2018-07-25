package us.ihmc.manipulation.planning.exploringSpatial;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ExploringDefinition
{
   private double trajectoryTime = 0.0;

   protected List<ExploringRigidBody> allExploringRigidBodies = new ArrayList<>();
   protected List<ReachingManifoldCommand> allReachingManifolds = new ArrayList<>();

   public ExploringDefinition(List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                              List<RigidBodyExplorationConfigurationCommand> explorationConfigurations, List<ReachingManifoldCommand> manifolds)
   {
      Map<RigidBody, WaypointBasedTrajectoryCommand> rigidBodyToTrajectoryMap = new HashMap<>();
      Map<RigidBody, RigidBodyExplorationConfigurationCommand> rigidBodyToExploringMap = new HashMap<>();
      Map<RigidBody, List<ReachingManifoldCommand>> rigidBodyToListOfManifoldMap = new HashMap<>();

      for (int i = 0; i < endEffectorTrajectories.size(); i++)
      {
         rigidBodyToTrajectoryMap.put(endEffectorTrajectories.get(i).getEndEffector(), endEffectorTrajectories.get(i));
         if (trajectoryTime < endEffectorTrajectories.get(i).getLastWaypointTime())
            trajectoryTime = endEffectorTrajectories.get(i).getLastWaypointTime();
      }

      for (int i = 0; i < explorationConfigurations.size(); i++)
         rigidBodyToExploringMap.put(explorationConfigurations.get(i).getRigidBody(), explorationConfigurations.get(i));

      for (int i = 0; i < manifolds.size(); i++)
         rigidBodyToListOfManifoldMap.put(manifolds.get(i).getRigidBody(), new ArrayList<ReachingManifoldCommand>());

      for (int i = 0; i < manifolds.size(); i++)
         rigidBodyToListOfManifoldMap.get(manifolds.get(i).getRigidBody()).add(manifolds.get(i));

      Set<RigidBody> rigidBodySet = new HashSet<>();
      rigidBodySet.addAll(rigidBodyToTrajectoryMap.keySet());
      rigidBodySet.addAll(rigidBodyToExploringMap.keySet());

      List<RigidBody> allRigidBodies = new ArrayList<>(rigidBodySet);

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);
         ExploringRigidBody exploringRigidBody = new ExploringRigidBody(rigidBody, rigidBodyToTrajectoryMap.get(rigidBody),
                                                                        rigidBodyToExploringMap.get(rigidBody), rigidBodyToListOfManifoldMap.get(rigidBody));
         allExploringRigidBodies.add(exploringRigidBody);
      }

      if (manifolds != null)
         allReachingManifolds.addAll(manifolds);
   }

   public SpatialData createDefaultSpatialData()
   {
      SpatialData spatialData = new SpatialData();
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
         allExploringRigidBodies.get(i).appendDefaultSpatialData(spatialData);

      return spatialData;
   }

   public SpatialData createRandomSpatialData()
   {
      SpatialData randomSpatialData = new SpatialData();
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
         allExploringRigidBodies.get(i).appendRandomSpatialData(randomSpatialData);

      return randomSpatialData;
   }

   public SpatialNode createFinalSpatialNode(SpatialNode lastNode)
   {
      double expectedNodeTime = Double.MAX_VALUE;
      SpatialData spatialData = new SpatialData();
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
      {
         double expectedTime = allExploringRigidBodies.get(i).appendFinalSpatialData(lastNode, trajectoryTime, spatialData);
         expectedNodeTime = Math.min(expectedNodeTime, expectedTime);
      }
      SpatialNode spatialNode = new SpatialNode(expectedNodeTime, spatialData);
      spatialNode.setParent(lastNode);
      return spatialNode;
   }

   public List<KinematicsToolboxRigidBodyMessage> createMessages(SpatialNode node)
   {
      List<KinematicsToolboxRigidBodyMessage> messages = new ArrayList<>();
      double timeInTrajectory = node.getTime();

      for (int i = 0; i < allExploringRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allExploringRigidBodies.get(i).getRigidBody();

         RigidBodyTransform poseToAppend = node.getSpatialData(rigidBody);

         KinematicsToolboxRigidBodyMessage message = allExploringRigidBodies.get(i).createMessage(timeInTrajectory, poseToAppend);
         messages.add(message);
      }

      return messages;
   }

   public double getExploringProgress(SpatialNode node)
   {
      if(!node.isValid())
         return 0.0;
      double progress = Double.MAX_VALUE;

      for (int i = 0; i < allExploringRigidBodies.size(); i++)
         progress = Math.min(progress, allExploringRigidBodies.get(i).getExploringProgress(node));

      return progress;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }
}
