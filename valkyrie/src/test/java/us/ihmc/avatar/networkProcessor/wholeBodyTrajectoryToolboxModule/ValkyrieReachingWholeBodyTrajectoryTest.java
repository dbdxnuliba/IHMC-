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
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class ValkyrieReachingWholeBodyTrajectoryTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
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

      Point3D sphereCenter = new Point3D(0.7, 0.2, 0.6);
      List<ReachingManifoldMessage> reachingManifoldMessages = ReachingManifoldTools.createSphereManifoldMessagesForValkyrie(robotSide, hand, sphereCenter,
                                                                                                                             0.1);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, hand, robotSide, reachingManifoldMessages);
      runTrajectoryTest(message, maxNumberOfIterations);

      PrintTools.info("END");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReachingTrajectoryTowardCylinder() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      RobotSide robotSide = RobotSide.LEFT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      Point3D center = new Point3D(0.7, 0.0, 0.6);
      RotationMatrix orientation = new RotationMatrix();
      orientation.appendPitchRotation(Math.PI * 0.3);
      List<ReachingManifoldMessage> reachingManifoldMessages = ReachingManifoldTools.createCylinderManifoldMessagesForValkyrie(robotSide, hand, center,
                                                                                                                               orientation, 0.1, 0.2);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, hand, robotSide, reachingManifoldMessages);
      runTrajectoryTest(message, maxNumberOfIterations);

      PrintTools.info("END");
   }
   
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReachingTrajectoryTowardTorus() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      RobotSide robotSide = RobotSide.LEFT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      Point3D center = new Point3D(0.8, 0.0, 1.1);
      RotationMatrix orientation = new RotationMatrix();
      orientation.appendPitchRotation(-Math.PI * 0.25);
      List<ReachingManifoldMessage> reachingManifoldMessages = ReachingManifoldTools.createTorusManifoldMessagesForValkyrie(robotSide, hand, center,
                                                                                                                               orientation, 0.3, 0.025);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, hand, robotSide, reachingManifoldMessages);
      runTrajectoryTest(message, maxNumberOfIterations);

      PrintTools.info("END");
   }

   private WholeBodyTrajectoryToolboxMessage createReachingWholeBodyTrajectoryToolboxMessage(FullHumanoidRobotModel fullRobotModel, RigidBody hand,
                                                                                             RobotSide robotSide,
                                                                                             List<ReachingManifoldMessage> reachingManifoldMessages)
   {
      if (VERBOSE)
      {
         OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();
         for (int i = 0; i < oneDoFJoints.length; i++)
            PrintTools.info("" + oneDoFJoints[i].getName() + " " + oneDoFJoints[i].getJointLimitUpper() + " " + oneDoFJoints[i].getJointLimitLower());
      }
      // input
      double extrapolateRatio = 1.5;
      double trajectoryTimeBeforeExtrapolated = 5.0;
      double trajectoryTime = trajectoryTimeBeforeExtrapolated * extrapolateRatio;

      // wbt toolbox configuration message
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      RigidBodyTransform handTransform = handControlFrame.getTransformToWorldFrame();

      RigidBodyTransform closestPointOnManifold = new RigidBodyTransform();
      RigidBodyTransform endTransformOnTrajectory = new RigidBodyTransform();

      List<ReachingManifoldCommand> manifolds = new ArrayList<>();
      for (int i = 0; i < reachingManifoldMessages.size(); i++)
      {
         ReachingManifoldCommand manifold = new ReachingManifoldCommand();
         manifold.setFromMessage(reachingManifoldMessages.get(i));
         manifolds.add(manifold);
      }
      ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(manifolds, handTransform, closestPointOnManifold, 1.0, 0.1);
      ReachingManifoldTools.packExtrapolatedTransform(handTransform, closestPointOnManifold, extrapolateRatio, endTransformOnTrajectory);
      reachingManifolds.addAll(reachingManifoldMessages);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeLinearTrajectory(time, trajectoryTime, handTransform, endTransformOnTrajectory);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.SE3};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);

      Graphics3DObject tempGraphic = new Graphics3DObject();
      tempGraphic.transform(closestPointOnManifold);
      tempGraphic.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(tempGraphic);
      
      PrintTools.info("default final transform");
      System.out.println(closestPointOnManifold);

      Graphics3DObject tempGraphic2 = new Graphics3DObject();
      tempGraphic2.transform(handTransform);
      tempGraphic2.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(tempGraphic2);

      Graphics3DObject tempGraphic3 = new Graphics3DObject();
      tempGraphic3.transform(endTransformOnTrajectory);
      tempGraphic3.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(tempGraphic3);

      return message;
   }
   
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testDoorMotion() throws Exception, UnreasonableAccelerationException
   {
      // trajectory parameter
      double trajectoryTime = 10.0;

      double openingAngle = 10.0 / 180.0 * Math.PI;
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
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeOpeningDoorTrajectory(time, trajectoryTime, openingRadius, openingAngle,
                                                                                                     openingDirectionCW, knobPose, twistTime, twistRadius,
                                                                                                     twistAngle, twistDirectionCW);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      RigidBodyTransform handControlFrameTransformToBodyFixedFrame = new RigidBodyTransform();
      handControlFrame.getTransformToDesiredFrame(handControlFrameTransformToBodyFixedFrame, hand.getBodyFixedFrame());
      trajectory.getControlFramePositionInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getTranslationVector());
      trajectory.getControlFrameOrientationInEndEffector().set(handControlFrameTransformToBodyFixedFrame.getRotationMatrix());

      handTrajectories.add(trajectory);

      // exploration configuration
      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      RigidBody chest = fullRobotModel.getChest();
      ConfigurationSpaceName[] chestExploringSpaces = {ConfigurationSpaceName.SE3};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(chest, chestExploringSpaces));

      // manifold
      ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createGoalManifoldMessage(hand, handFunction, trajectoryTime, spaces);
      reachingManifolds.add(reachingManifold);

      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }
}
