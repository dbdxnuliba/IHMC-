package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class ExploringRigidBody
{
   private RigidBody rigidBody;

   private List<ExploringConfigurationSpace> exploringConfigurationSpaces = new ArrayList<ExploringConfigurationSpace>();

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final ArrayList<Pose3D> waypoints = new ArrayList<Pose3D>();

   private final SelectionMatrix6D trajectorySelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D explorationSelectionMatrix = new SelectionMatrix6D();

   private double weight;
   private static double DEFAULT_WEIGHT = 20.0;

   /**
    * For exploring rigid body without constrained trajectory.
    * Its' way point would be current pose of full robot model.
    */
   public ExploringRigidBody(RigidBody rigidBody, RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      this(rigidBody, null, explorationCommand);
   }

   /**
    * Not exploring rigid body.
    * It will return initial pose only.
    */
   public ExploringRigidBody(RigidBody rigidBody, WaypointBasedTrajectoryCommand trajectoryCommand)
   {
      this(rigidBody, trajectoryCommand, null);
   }

   /**
    * For exploring rigid body with constrained trajectory.
    * Exploring configuration space would be appended on the trajectory.
    */
   public ExploringRigidBody(RigidBody rigidBody, WaypointBasedTrajectoryCommand trajectoryCommand, RigidBodyExplorationConfigurationCommand explorationCommand)
   {
      this.rigidBody = rigidBody;

      // trajectory command.
      if (trajectoryCommand == null)
      {
         waypointTimes.add(0.0);
         waypointTimes.add(Double.MAX_VALUE);
         Pose3D originOfRigidBody = new Pose3D(rigidBody.getBodyFixedFrame().getTransformToWorldFrame());
         waypoints.add(originOfRigidBody);
         waypoints.add(originOfRigidBody);
         trajectorySelectionMatrix.clearSelection();
         weight = DEFAULT_WEIGHT;
      }
      else
      {

         for (int i = 0; i < trajectoryCommand.getNumberOfWaypoints(); i++)
         {
            waypointTimes.add(trajectoryCommand.getWaypointTime(i));
            waypoints.add(new Pose3D(trajectoryCommand.getWaypoint(i)));
         }
         trajectorySelectionMatrix.set(trajectoryCommand.getSelectionMatrix());

         if (Double.isNaN(trajectoryCommand.getWeight()) || trajectoryCommand.getWeight() < 0.0)
            weight = DEFAULT_WEIGHT;
         else
            weight = trajectoryCommand.getWeight();
      }

      // exploration command.
      explorationSelectionMatrix.clearSelection();
      for (int i = 0; i < explorationCommand.getNumberOfDegreesOfFreedomToExplore(); i++)
      {
         ExploringConfigurationSpace exploringConfigurationSpace = new ExploringConfigurationSpace(explorationCommand.getDegreeOfFreedomToExplore(i),
                                                                                                   explorationCommand.getExplorationRangeLowerLimits(i),
                                                                                                   explorationCommand.getExplorationRangeUpperLimits(i));

         setSelectionMatrix(explorationSelectionMatrix, explorationCommand.getDegreeOfFreedomToExplore(i), true);

         // TODO : should be used as default.
         if (explorationCommand.getDegreeOfFreedomToExplore(i) == ConfigurationSpaceName.SE3)
         {
            exploringConfigurationSpace.setLowerLimit(0.0);
            exploringConfigurationSpace.setUpperLimit(1.0);
         }

         exploringConfigurationSpaces.add(exploringConfigurationSpace);
      }
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   // TODO : temporary to use appendSpatial in SpatialData.
   public void appendRandomSpatialData(SpatialData spatialData)
   {
      String rigidBodyName = rigidBody.getName();
      String[] configurationNames = new String[exploringConfigurationSpaces.size()];
      double[] configurationData = new double[exploringConfigurationSpaces.size()];
      RigidBodyTransform pose = new RigidBodyTransform();

      for (int i = 0; i < exploringConfigurationSpaces.size(); i++)
      {
         configurationNames[i] = rigidBody + "_" + exploringConfigurationSpaces.get(i).getConfigurationSpaceName().name();
         configurationData[i] = exploringConfigurationSpaces.get(i).getConfiguration();
         pose.transform(exploringConfigurationSpaces.get(i).getRandomLocalRigidBodyTransform());
      }

      spatialData.appendSpatial(rigidBodyName, configurationNames, configurationData, pose);
   }

   private void setSelectionMatrix(SelectionMatrix6D selectionMatrix, ConfigurationSpaceName configurationSpaceName, boolean select)
   {
      switch (configurationSpaceName)
      {
      case X:
         selectionMatrix.selectLinearX(select);
         break;
      case Y:
         selectionMatrix.selectLinearY(select);
         break;
      case Z:
         selectionMatrix.selectLinearZ(select);
         break;
      case ROLL:
         selectionMatrix.selectAngularX(select);
         break;
      case PITCH:
         selectionMatrix.selectAngularY(select);
         break;
      case YAW:
         selectionMatrix.selectAngularZ(select);
         break;
      case SE3:
         selectionMatrix.selectAngularX(select);
         selectionMatrix.selectAngularY(select);
         selectionMatrix.selectAngularZ(select);
         break;
      default:
         throw new RuntimeException("Unexpected enum value: " + configurationSpaceName);
      }
   }

   private SelectionMatrix6D getSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();

      if (trajectorySelectionMatrix.getLinearPart().isXSelected() || explorationSelectionMatrix.getLinearPart().isXSelected())
         selectionMatrix.getLinearPart().selectXAxis(true);

      if (trajectorySelectionMatrix.getLinearPart().isYSelected() || explorationSelectionMatrix.getLinearPart().isYSelected())
         selectionMatrix.getLinearPart().selectYAxis(true);

      if (trajectorySelectionMatrix.getLinearPart().isZSelected() || explorationSelectionMatrix.getLinearPart().isZSelected())
         selectionMatrix.getLinearPart().selectZAxis(true);

      if (trajectorySelectionMatrix.getAngularPart().isXSelected() || explorationSelectionMatrix.getAngularPart().isXSelected())
         selectionMatrix.getAngularPart().selectXAxis(true);

      if (trajectorySelectionMatrix.getAngularPart().isYSelected() || explorationSelectionMatrix.getAngularPart().isYSelected())
         selectionMatrix.getAngularPart().selectYAxis(true);

      if (trajectorySelectionMatrix.getAngularPart().isZSelected() || explorationSelectionMatrix.getAngularPart().isZSelected())
         selectionMatrix.getAngularPart().selectZAxis(true);

      return selectionMatrix;
   }
}
