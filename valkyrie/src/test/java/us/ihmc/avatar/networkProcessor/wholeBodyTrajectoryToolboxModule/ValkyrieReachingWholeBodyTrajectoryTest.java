package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;

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
import us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule.AvatarWholeBodyTrajectoryToolboxControllerTest;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
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
   /**
    *  TODO : Implement {@link ExploringDefinitionToReachingManifold}.
    */
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReaching() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(2300);

      RigidBody hand = fullRobotModel.getHand(RobotSide.RIGHT);
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      ReachingManifoldMessage reachingManifold = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifold.getManifoldOriginPosition().set(new Point3D(0.7, -0.2, 1.0));
      reachingManifold.getManifoldOriginOrientation().set(new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {YAW, PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI * 0.5, -Math.PI * 0.5, -0.1};
      double[] upperLimits = new double[] {Math.PI * 0.5, Math.PI * 0.5, 0.0};
      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifold);
      reachingManifolds.add(reachingManifold);

      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      /**
       * BIT   @link https://arxiv.org/pdf/1405.5848.pdf
       * RABIT @link https://www.ri.cmu.edu/pub_files/2016/5/main.pdf.
       * Implement BIT, RABIT for adaptive random regions.
       */
      // test for position only.
      ConfigurationSpaceName[] explorationSpaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,};
      double[] explorationUpperLimits = {0.15, 0.05, 0.2};
      double[] explorationLowerLimits = {-0.0, -0.5, -0.2};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, explorationSpaces, explorationUpperLimits,
                                                                                                      explorationLowerLimits));

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, null, reachingManifolds,
                                                                                                               rigidBodyConfigurations);

      // run toolbox
      runReachingTest(message, maxNumberOfIterations);

      PrintTools.info("END");
   }
}
