package us.ihmc.avatar.networkProcessor.rrTToolboxModule;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.AvatarWholeBodyTrajectoryToolboxControllerTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class ValkyrieConstrainedWholeBodyTrajectoryPlanningTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
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

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 10.6)
   @Test(timeout = 53000)
   public void testOneBigCircle() throws Exception, UnreasonableAccelerationException
   {
      super.testOneBigCircle();
   }
   
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 10.6)
   @Test(timeout = 53000)
   public void testDrumLifting() throws Exception, UnreasonableAccelerationException
   {
      // Trajectory params.
      double trajectoryTime = 5.0;
      Point3D holdingPosition = new Point3D(0.6, -0.3, 1.0);
      Quaternion holdingOrientation = new Quaternion();
      holdingOrientation.appendYawRotation(Math.PI*0.1);

      // WBT toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // Trajectory message, Exploration message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == RobotSide.RIGHT)
         {
            RigidBody hand = fullRobotModel.getHand(robotSide);

            FunctionTrajectory handFunction = time -> new Pose3D(holdingPosition, holdingOrientation);

            SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
            selectionMatrix.resetSelection();
            WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, handFunction, selectionMatrix);
            
            Pose3D controlFramePose = new Pose3D(fullRobotModel.getHandControlFrame(robotSide).getTransformToParent());

            trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
            trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());
            
            handTrajectories.add(trajectory);
            ConfigurationSpaceName[] handConfigurations = {};
            RigidBodyExplorationConfigurationMessage rigidBodyConfiguration = HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand,
                                                                                                                                                  handConfigurations);

            rigidBodyConfigurations.add(rigidBodyConfiguration);

            if (visualize)
               scs.addStaticLinkGraphics(createFunctionTrajectoryVisualization(handFunction, 0.0, trajectoryTime, 0.1, 0.01, YoAppearance.AliceBlue()));
         }
      }
      
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null,
                                                                                                               rigidBodyConfigurations);

      // run toolbox
      runTrajectoryTest(message, 100000);
   }
}
