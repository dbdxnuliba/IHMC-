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
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
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

   private void packExtrapolatedPoint(Vector3DReadOnly from, Vector3DReadOnly to, double ratio, Point3D toPack)
   {
      toPack.setX(ratio * (to.getX() - from.getX()) + from.getX());
      toPack.setY(ratio * (to.getY() - from.getY()) + from.getY());
      toPack.setZ(ratio * (to.getZ() - from.getZ()) + from.getZ());

      System.out.println(from.getX() + " " + from.getY() + " " + from.getZ());
      System.out.println(to.getX() + " " + to.getY() + " " + to.getZ());
      System.out.println(toPack.getX() + " " + toPack.getY() + " " + toPack.getZ());
   }

   private void packExtrapolatedOrienation(RotationMatrixReadOnly from, RotationMatrixReadOnly to, double ratio, RotationMatrix toPack)
   {
      Quaternion invFrom = new Quaternion(from);
      invFrom.inverse();

      Quaternion delFromTo = new Quaternion();
      delFromTo.multiply(invFrom, new Quaternion(to));

      AxisAngle delFromToAxisAngle = new AxisAngle();
      AxisAngleConversion.convertQuaternionToAxisAngle(delFromTo, delFromToAxisAngle);

      PrintTools.info("");
      System.out.println(delFromToAxisAngle.getAngle() + " " + delFromToAxisAngle.getX() + " " + delFromToAxisAngle.getY() + " " + delFromToAxisAngle.getZ());

      AxisAngle delFromExtraAxisAngle = new AxisAngle(delFromToAxisAngle);
      double extrapolatedAngle = ratio * delFromToAxisAngle.getAngle();
      delFromExtraAxisAngle.setAngle(extrapolatedAngle);

      AxisAngle toPackAxisAngle = new AxisAngle(from);
//      System.out.println(toPackAxisAngle.getAngle() + " " + toPackAxisAngle.getX() + " " + toPackAxisAngle.getY() + " " + toPackAxisAngle.getZ());
//      AxisAngle toAxisAngle = new AxisAngle(to);
//      System.out.println(toAxisAngle.getAngle() + " " + toAxisAngle.getX() + " " + toAxisAngle.getY() + " " + toAxisAngle.getZ());
      toPackAxisAngle.multiply(delFromExtraAxisAngle);
//      System.out.println(toPackAxisAngle.getAngle() + " " + toPackAxisAngle.getX() + " " + toPackAxisAngle.getY() + " " + toPackAxisAngle.getZ());

      AxisAngle temp = new AxisAngle(from);
      temp.multiply(delFromToAxisAngle);
//      System.out.println(temp.getAngle() + " " + temp.getX() + " " + temp.getY() + " " + temp.getZ());

      RotationMatrixConversion.convertAxisAngleToMatrix(toPackAxisAngle, toPack);
   }

   private void packExtrapolatedTransform(RigidBodyTransform from, RigidBodyTransform to, double ratio, RigidBodyTransform toPack)
   {
      Point3D pointToPack = new Point3D();
      RotationMatrix orientationToPack = new RotationMatrix();

      packExtrapolatedPoint(from.getTranslationVector(), to.getTranslationVector(), ratio, pointToPack);
      packExtrapolatedOrienation(from.getRotationMatrix(), to.getRotationMatrix(), ratio, orientationToPack);

      toPack.setIdentity();
      toPack.setTranslation(pointToPack);
      toPack.setRotation(orientationToPack);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReachingExtrapolatedLinearTrajectory() throws Exception, UnreasonableAccelerationException
   {
      // input
      double extrapolateRatio = 1.5;
      double trajectoryTimeBeforeExtrapolated = 5.0;

      double trajectoryTime = trajectoryTimeBeforeExtrapolated * extrapolateRatio;
      Point3D sphereCenter = new Point3D(0.7, -0.2, 0.6);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(500);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);
      RigidBodyTransform handTransform = fullRobotModel.getHandControlFrame(RobotSide.RIGHT).getTransformToWorldFrame();

      RigidBodyTransform closestPointOnManifold = new RigidBodyTransform();
      RigidBodyTransform endTransformOnTrajectory = new RigidBodyTransform();

      ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createSphereManifoldMessage(hand, sphereCenter, 0.1);
      ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(reachingManifold, handTransform, closestPointOnManifold);
      System.out.println(closestPointOnManifold);
      packExtrapolatedTransform(handTransform, closestPointOnManifold, extrapolateRatio, endTransformOnTrajectory);
      
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(closestPointOnManifold);
      graphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(graphics);

      reachingManifolds.add(reachingManifold);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeLinearTrajectory(time, trajectoryTime, handTransform, endTransformOnTrajectory);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);
      Pose3D controlFramePose = new Pose3D(fullRobotModel.getHandControlFrame(robotSide).getTransformToParent());

      trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
      trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.SE3};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      RigidBody chest = fullRobotModel.getChest();
      ConfigurationSpaceName[] chestExploringSpaces = {ConfigurationSpaceName.SE3};
      //rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(chest, chestExploringSpaces));
      
      // To hold another side hand.
      RigidBody anotherHand = fullRobotModel.getHand(robotSide.getOppositeSide());
      //rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(anotherHand));

      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories,
                                                                                                               reachingManifolds, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);

      PrintTools.info("END");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testDoorMotion() throws Exception, UnreasonableAccelerationException
   {
      // trajectory parameter
      double trajectoryTime = 5.0;

      double openingAngle = 30.0 / 180.0 * Math.PI;
      double openingRadius = 0.8;
      boolean openingDirectionCW = true; // in X-Y plane.

      //Point3D knobPosition = new Point3D(0.6, -0.25, 1.0);
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
      configuration.setMaximumExpansionSize(500);

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
      Pose3D controlFramePose = new Pose3D(fullRobotModel.getHandControlFrame(robotSide).getTransformToParent());

      trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
      trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

      handTrajectories.add(trajectory);

      // exploration configuration
      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      RigidBody chest = fullRobotModel.getChest();
      ConfigurationSpaceName[] chestExploringSpaces = {ConfigurationSpaceName.SE3};
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(chest, chestExploringSpaces));

      RigidBody anotherHand = fullRobotModel.getHand(robotSide.getOppositeSide());
      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(anotherHand));
      
      // manifold
      ReachingManifoldMessage reachingManifold = ReachingManifoldTools.createGoalManifoldMessage(hand, handFunction, trajectoryTime, spaces);
      reachingManifolds.add(reachingManifold);
      
      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, reachingManifolds,
                                                                                                               rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }
}
