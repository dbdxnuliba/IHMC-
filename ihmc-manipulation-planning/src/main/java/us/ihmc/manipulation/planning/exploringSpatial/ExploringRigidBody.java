package us.ihmc.manipulation.planning.exploringSpatial;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

/**
 * If ExploringRigidBody has no exploring configuration, it will return the initial pose only.
 * If ExploringRigidBody has no trajectory, it will explore freely starting from initial pose.
 * If ExploringRigidBody has goal manifold, it will return waypointTimeOnManifold.
 */
public class ExploringRigidBody
{
   private RigidBody rigidBody;

   private List<ExploringConfigurationSpace> exploringConfigurationSpaces = new ArrayList<ExploringConfigurationSpace>();

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final ArrayList<Pose3D> waypoints = new ArrayList<Pose3D>();

   private Pose3D currentPose = new Pose3D();

   private final Pose3D controlFramePose = new Pose3D();

   private final SelectionMatrix6D trajectorySelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D explorationSelectionMatrix = new SelectionMatrix6D();

   private double weight;
   private static double DEFAULT_WEIGHT = 20.0;

   private final List<ReachingManifoldCommand> manifolds = new ArrayList<>();

   private final double initialDistanceToManifold;

   private static final double positionWeight = 1.0;
   private static final double orientationWeight = 0.1;

   /**
    * For exploring rigid body with constrained trajectory.
    * Exploring configuration space would be appended on the trajectory.
    */
   public ExploringRigidBody(RigidBody rigidBody, WaypointBasedTrajectoryCommand trajectoryCommand, RigidBodyExplorationConfigurationCommand explorationCommand,
                             List<ReachingManifoldCommand> manifolds)
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

         exploringConfigurationSpaces.add(exploringConfigurationSpace);
      }

      // manifold commands.
      if (manifolds == null)
      {
         initialDistanceToManifold = 0.0;
      }
      else
      {
         this.manifolds.addAll(manifolds);
         Pose3D initialPose = appendPoseToTrajectory(0.0, new RigidBodyTransform());
         RigidBodyTransform closestTransform = new RigidBodyTransform();
         initialDistanceToManifold = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(this.manifolds, initialPose, closestTransform,
                                                                                                   positionWeight, orientationWeight);
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

   public double getExploringProgress(SpatialNode spatialNode)
   {
      if (manifolds.size() == 0)
         return Double.MAX_VALUE;

      double nodeTime = spatialNode.getTime();

      RigidBodyTransform spatial = spatialNode.getSpatialData(rigidBody);
      Pose3D poseToWorld = appendPoseToTrajectory(nodeTime, spatial);

      RigidBodyTransform closestTransform = new RigidBodyTransform();

      double distance = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifolds, poseToWorld, closestTransform, positionWeight,
                                                                                      orientationWeight);

      return 1 - distance / initialDistanceToManifold;
   }

   public void appendDefaultSpatialData(SpatialData spatialData)
   {
      spatialData.addSpatial(rigidBody.getName(), new RigidBodyTransform());
   }

   public void appendRandomSpatialData(SpatialData spatialData)
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int i = 0; i < exploringConfigurationSpaces.size(); i++)
         exploringConfigurationSpaces.get(i).getRandomLocalRigidBodyTransform().transform(transform);

      spatialData.addSpatial(rigidBody.getName(), transform);
   }

   public double appendFinalSpatialData(SpatialNode lastNode, double trajectoryTime, SpatialData spatialDataToAppend)
   {
      if (manifolds.size() == 0)
      {
         RigidBodyTransform spatialToAppendClosestTransformToManifolds = new RigidBodyTransform();
         double nodeTime = lastNode.getTime();
         double parentNodeTime = lastNode.getParent().getTime();
         double extrapolateRatio = 2.0;

         double expectedReachingTime = parentNodeTime + extrapolateRatio * (nodeTime - parentNodeTime);

         RigidBodyTransform from = lastNode.getParent().getSpatialData(rigidBody);
         RigidBodyTransform to = lastNode.getSpatialData(rigidBody);

         ReachingManifoldTools.packExtrapolatedTransform(from, to, extrapolateRatio, spatialToAppendClosestTransformToManifolds);
         spatialDataToAppend.addSpatial(rigidBody.getName(), spatialToAppendClosestTransformToManifolds);
         
         PrintTools.info(""+expectedReachingTime);
         
         return expectedReachingTime;
      }
      else
      {
         RigidBodyTransform lastSpatial = lastNode.getSpatialData(rigidBody);
         Pose3D lastPoseToWorld = appendPoseToTrajectory(lastNode.getTime(), lastSpatial);

         RigidBodyTransform closestTransformToLastNode = new RigidBodyTransform();
         double lastDistance = ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifolds, lastPoseToWorld, closestTransformToLastNode,
                                                                                             positionWeight, orientationWeight);

         double expectedReachingTime = lastNode.getTime() * (initialDistanceToManifold) / (initialDistanceToManifold - lastDistance);
         Pose3D expectedPose = getCurrentPose(expectedReachingTime);

         RigidBodyTransform expectedHandTransform = new RigidBodyTransform(expectedPose.getOrientation(), expectedPose.getPosition());

         RigidBodyTransform transformHandtoManifold = new RigidBodyTransform(closestTransformToLastNode);
         transformHandtoManifold.preMultiplyInvertOther(expectedHandTransform);

         spatialDataToAppend.addSpatial(rigidBody.getName(), transformHandtoManifold);

         PrintTools.info("final transform");
         System.out.println(closestTransformToLastNode);

         return expectedReachingTime;
      }
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

   private Pose3D getCurrentPose(double time)
   {
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
            break;
      }

      double alpha = (time - t0) / (tf - t0);
      alpha = MathTools.clamp(alpha, 0, 1);

      currentPose.interpolate(previous, next, alpha);

      return currentPose;
   }

   private Pose3D appendPoseToTrajectory(double timeInTrajectory, RigidBodyTransform transformToAppend)
   {
      Pose3D pose = getCurrentPose(timeInTrajectory);

      pose.appendTransform(transformToAppend);

      return pose;
   }
}
