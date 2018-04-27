package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
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

   private final Pose3D controlFramePose = new Pose3D();

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

         // TODO : for reaching, since there is no trajectory command, we should put proper hand control frame on this.         
         controlFramePose.setToZero();

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

         FramePose3D controlFramePose = new FramePose3D(trajectoryCommand.getControlFramePose());
         controlFramePose.changeFrame(trajectoryCommand.getEndEffector().getBodyFixedFrame());
         this.controlFramePose.set(controlFramePose);

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

   public List<ExploringConfigurationSpace> getExploringConfigurationSpaces()
   {
      return exploringConfigurationSpaces;
   }

   public void appendRandomSpatialData(SpatialData spatialData)
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < exploringConfigurationSpaces.size(); i++)
         exploringConfigurationSpaces.get(i).getRandomLocalRigidBodyTransform().transform(transform);

      spatialData.addSpatial(rigidBody.getName(), exploringConfigurationSpaces, transform);
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

   private Pose3D getPose(double time)
   {
      Pose3D current = new Pose3D();

      Pose3D previous = null;
      Pose3D next = null;
      double t0 = Double.NaN;
      double tf = Double.NaN;

      for (int i = 1; i < waypoints.size(); i++)
      {
         t0 = waypointTimes.get(i - 1);
         tf = waypointTimes.get(i);
         previous = waypoints.get(i - 1);
         next = waypoints.get(i);
         if (time < tf)
         {
            break;
         }
      }

      double alpha = (time - t0) / (tf - t0);
      alpha = MathTools.clamp(alpha, 0, 1);

      current.interpolate(previous, next, alpha);

      return current;
   }

   private Pose3D appendPoseToTrajectory(double timeInTrajectory, RigidBodyTransform transformToAppend)
   {
      Pose3D pose = getPose(timeInTrajectory);

      pose.appendTransform(transformToAppend);

      return pose;
   }

   public KinematicsToolboxRigidBodyMessage createMessage(double timeInTrajectory, RigidBodyTransform poseToAppend)
   {
      Pose3D desiredEndEffectorPose = appendPoseToTrajectory(timeInTrajectory, poseToAppend);

      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody);
      message.getDesiredPositionInWorld().set(desiredEndEffectorPose.getPosition());
      message.getDesiredOrientationInWorld().set(desiredEndEffectorPose.getOrientation());
      message.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
      message.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());
      message.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(getSelectionMatrix().getAngularPart()));
      message.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(getSelectionMatrix().getLinearPart()));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(weight));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(weight));

      return message;
   }
}
