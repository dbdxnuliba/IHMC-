package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class ExternalForceEstimationToolboxController extends ToolboxController
{
   private final ExternalForceEstimator externalForceEstimator;
   private final CommandInputManager commandInputManager;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();

   public ExternalForceEstimationToolboxController(DRCRobotModel robotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;

      this.fullRobotModel = robotModel.createFullRobotModel();
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      JointBasics[] jointsToIgnore = new JointBasics[0]; // new JointBasics[]{controllerFullRobotModel.getOneDoFJointByName("hokuyo_joint")}; //
      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(robotModel.getControllerDT(),
                                                                                       9.81,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       joints,
                                                                                       referenceFrames.getCenterOfMassFrame(),
                                                                                       robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings(),
                                                                                       graphicsListRegistry,
                                                                                       parentRegistry);
      // TODO set up end effector configuration through a message
      Vector3D externalForcePointOffset = new Vector3D(0.0, -0.3, 0.0);
      RigidBodyBasics endEffector = fullRobotModel.getOneDoFJointByName("rightForearmYaw").getSuccessor();

      this.externalForceEstimator = new ExternalForceEstimator(joints, endEffector, externalForcePointOffset, );
   }

   @Override
   public boolean initialize()
   {
      externalForceEstimator.initialize();
      return true;
   }

   @Override
   public void updateInternal() throws Exception
   {

   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   public void updateRobotDesiredConfigurationData(RobotDesiredConfigurationData robotDesiredConfigurationData)
   {
      this.robotDesiredConfigurationData.set(robotDesiredConfigurationData);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
