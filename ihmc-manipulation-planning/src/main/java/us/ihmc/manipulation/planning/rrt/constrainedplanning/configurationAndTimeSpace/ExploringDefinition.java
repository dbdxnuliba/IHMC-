package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.robotics.screwTheory.RigidBody;

public abstract class ExploringDefinition
{
   private List<ExploringRigidBody> allExploringRigidBodies = new ArrayList<>();

   public ExploringDefinition(List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                              List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      Map<RigidBody, WaypointBasedTrajectoryCommand> rigidBodyToTrajectoryMap = new HashMap<>();
      Map<RigidBody, RigidBodyExplorationConfigurationCommand> rigidBodyToExploringMap = new HashMap<>();

      for (int i = 0; i < endEffectorTrajectories.size(); i++)
         rigidBodyToTrajectoryMap.put(endEffectorTrajectories.get(i).getEndEffector(), endEffectorTrajectories.get(i));

      for (int i = 0; i < explorationConfigurations.size(); i++)
         rigidBodyToExploringMap.put(explorationConfigurations.get(i).getRigidBody(), explorationConfigurations.get(i));

      Set<RigidBody> rigidBodySet = new HashSet<>();
      rigidBodySet.addAll(rigidBodyToTrajectoryMap.keySet());
      rigidBodySet.addAll(rigidBodyToExploringMap.keySet());

      List<RigidBody> allRigidBodies = new ArrayList<>(rigidBodySet);

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         PrintTools.info("" + allRigidBodies.get(i).getName());

         ExploringRigidBody exploringRigidBody = new ExploringRigidBody(allRigidBodies.get(i), rigidBodyToTrajectoryMap.get(allRigidBodies.get(i)),
                                                                        rigidBodyToExploringMap.get(allRigidBodies.get(i)));
         allExploringRigidBodies.add(exploringRigidBody);
      }
   }
   
   public SpatialData getRandomSpatialData()
   {
      SpatialData randomSpatialData = new SpatialData();
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
      {
         allExploringRigidBodies.get(i).appendRandomSpatialData(randomSpatialData);
      }

      return randomSpatialData;
   }

   public List<KinematicsToolboxRigidBodyMessage> createMessages(SpatialNode node)
   {
      List<KinematicsToolboxRigidBodyMessage> messages = new ArrayList<>();
      double timeInTrajectory = node.getTime();
      
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allExploringRigidBodies.get(i).getRigidBody();

         Pose3D poseToAppend = node.getSpatialData(rigidBody);

         KinematicsToolboxRigidBodyMessage message = allExploringRigidBodies.get(i).createMessage(timeInTrajectory, poseToAppend);
         messages.add(message);
      }

      return messages;
   }
   
   public abstract double getExploringProgress(SpatialNode node);
   
   public abstract double getTrajectoryTime();
}
