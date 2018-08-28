package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class ValkyrieWholeBodyTrajectoryToolboxTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReachingTrajectoryTowardSphere() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      RobotSide robotSide = RobotSide.LEFT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      double radius = 0.1;
      Point3D sphereCenter = new Point3D(0.7, 0.2, 0.6);
      List<ReachingManifoldMessage> reachingManifoldMessages = ReachingManifoldTools.createSphereManifoldMessagesForValkyrie(robotSide, hand, sphereCenter,
                                                                                                                             radius);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, hand, robotSide, reachingManifoldMessages);
      runTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReachingTrajectoryTowardCylinder() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      RobotSide robotSide = RobotSide.LEFT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      double radius = 0.1;
      double height = 0.2;
      Point3D center = new Point3D(0.7, 0.0, 0.6);
      RotationMatrix orientation = new RotationMatrix();
      orientation.appendPitchRotation(Math.PI * 0.3);
      List<ReachingManifoldMessage> reachingManifoldMessages = ReachingManifoldTools.createCylinderManifoldMessagesForValkyrie(robotSide, hand, center,
                                                                                                                               orientation, radius, height);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, hand, robotSide, reachingManifoldMessages);
      runTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReachingTrajectoryTowardTorus() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      RobotSide robotSide = RobotSide.LEFT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      double radius = 0.3;
      double tubeRadius = 0.025;
      Point3D center = new Point3D(0.8, 0.0, 1.1);
      RotationMatrix orientation = new RotationMatrix();
      orientation.appendPitchRotation(-Math.PI * 0.25);
      List<ReachingManifoldMessage> reachingManifoldMessages = ReachingManifoldTools.createTorusManifoldMessagesForValkyrie(robotSide, hand, center,
                                                                                                                            orientation, radius, tubeRadius);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, hand, robotSide, reachingManifoldMessages);
      runTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testDoorMotion() throws Exception, UnreasonableAccelerationException
   {
      // trajectory parameter
      double trajectoryTime = 10.0;

      double openingAngle = 30.0 / 180.0 * Math.PI;
      double openingRadius = 0.8;
      boolean openingDirectionCW = true; // in X-Y plane.

      Point3D knobPosition = new Point3D(0.45, -0.35, 1.0);
      Quaternion knobOrientation = new Quaternion();
      knobOrientation.appendYawRotation(-0.02 * Math.PI);
      knobOrientation.appendRollRotation(-0.5 * Math.PI);
      Pose3D knobPose = new Pose3D(knobPosition, knobOrientation); // grasping pose

      double twistTime = 1.0;
      double twistRadius = 0.15;
      double twistAngle = 60.0 / 180.0 * Math.PI;
      boolean twistDirectionCW = true; // plane which is parallel with knobDirection

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // toolbox messages
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      // trajectory.
      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);
      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeOpeningDoorTrajectory(time, trajectoryTime, openingRadius, openingAngle,
                                                                                                     openingDirectionCW, knobPose, twistTime, twistRadius,
                                                                                                     twistAngle, twistDirectionCW);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      handTrajectories.add(trajectory);

      // exploration configuration
      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // manifold
      ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createGoalManifoldMessage(hand, handFunction, trajectoryTime, spaces);
      reachingManifolds.add(reachingManifold);

      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);
      runTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testDrillMotion() throws Exception, UnreasonableAccelerationException
   {
      // trajectory parameter
      double trajectoryTime = 30.0;

      boolean cuttingDirectionCW = true;
      double cuttingRadius = 0.2;
      Vector3D wallNormalVector = new Vector3D(-1.0, 0.0, 0.0);
      Point3D cuttingCenterPosition = new Point3D(0.6, -0.3, 1.2);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> trajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeCuttingWallTrajectory(time, trajectoryTime, cuttingRadius, cuttingDirectionCW,
                                                                                                     cuttingCenterPosition, wallNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      trajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // keep sight on trajectory.
      RigidBody head = fullRobotModel.getHead();
      SelectionMatrix6D selectionMatrixHead = new SelectionMatrix6D();
      selectionMatrixHead.clearSelection();
      selectionMatrixHead.selectLinearY(true);
      selectionMatrixHead.selectLinearZ(true);

      WaypointBasedTrajectoryMessage trajectoryHead = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(head, 0.0, trajectoryTime, timeResolution,
                                                                                                                     handFunction, selectionMatrixHead);

      trajectoryHead.getControlFramePositionInEndEffector().set(new Point3D(0.5, 0.0, 0.0));
      trajectoryHead.setWeight(0.01);
      //trajectories.add(trajectoryHead);

      // manifold
      ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createGoalManifoldMessage(hand, handFunction, trajectoryTime, spaces);
      reachingManifolds.add(reachingManifold);

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, trajectories, reachingManifolds,
                                                                                                               rigidBodyConfigurations);
      runTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testValveMotion() throws Exception, UnreasonableAccelerationException
   {
      // trajectory parameter
      double trajectoryTime = 5.0;
      boolean closingDirectionCW = false;
      double closingRadius = 0.2;
      double closingAngle = Math.PI;
      Vector3D valveNormalVector = new Vector3D(-1.0, 0.3, 0.0);
      Point3D valveCenterPosition = new Point3D(0.5, -0.5, 1.1);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // toolbox messages
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      // trajectory
      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeClosingValveTrajectory(time, trajectoryTime, closingRadius, closingDirectionCW,
                                                                                                      closingAngle, valveCenterPosition, valveNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);
      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      handTrajectories.add(trajectory);

      // exploration configuration
      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // manifold
      ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createGoalManifoldMessage(hand, handFunction, trajectoryTime, spaces);
      reachingManifolds.add(reachingManifold);

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);
      runTest(message, maxNumberOfIterations);
   }
}