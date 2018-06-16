package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl.FootController;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MotionControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.DoubleSupportMotionState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HumanoidMotionController implements HighLevelHumanoidControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JumpControllerParameters controllerParamaters;

   // Control modules and such
   private final StateMachine<MotionControllerStateEnum, MotionControllerState> stateMachine;
   private final YoDouble yoTime;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final FullHumanoidRobotModel fullRobotModel;
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(PlaneContactStateCommand.class);

   private final List<ControlManagerInterface> controlManagerList = new ArrayList<>();
   private final List<RigidBodyControlManager> rigidBodyManagers = new ArrayList<>();
   private final SideDependentList<FootController> feetControllers = new SideDependentList<>();
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();
   private RigidBodyControlManager headManager;
   private RigidBodyControlManager chestManager;

   private final YoEnum<MotionControllerStateEnum> requestedState;
   // IO objects
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   public HumanoidMotionController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                   HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters motionControllerParameters,
                                   MotionControlManagerFactory motionControlManagerFactory, YoVariableRegistry parentRegistry)
   {
      controllerParamaters = motionControllerParameters;
      String name = "motionController";
      this.controllerToolbox = controllerToolbox;
      this.yoTime = controllerToolbox.getYoTime();
      fullRobotModel = controllerToolbox.getFullRobotModel();
      createRigidBodyControlManagers(controllerToolbox, motionControlManagerFactory);
      requestedState = new YoEnum<>(name + "NextState", registry, MotionControllerStateEnum.class);
      stateMachine = setupStateMachine();
      parentRegistry.addChild(registry);
   }

   private void createRigidBodyControlManagers(HighLevelHumanoidControllerToolbox controllerToolbox, MotionControlManagerFactory motionControlManagerFactory)
   {
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      RigidBody pelvis = fullRobotModel.getPelvis();
      if (pelvis == null)
         throw new RuntimeException("Humanoid robot must have a pelvis rigid body");
      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      // Create chest manager
      RigidBody chest = fullRobotModel.getChest();
      if (chest == null)
         throw new RuntimeException("Humanoid robot must have a chest rigid body");
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();
      chestManager = motionControlManagerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestFrame, pelvisFrame, trajectoryFrames);
      rigidBodyManagers.add(chestManager);

      // Create head manager
      RigidBody head = fullRobotModel.getHead();
      if (head != null)
      {
         ReferenceFrame headFrame = head.getBodyFixedFrame();
         headManager = motionControlManagerFactory.getOrCreateRigidBodyManager(head, chest, headFrame, chestFrame, trajectoryFrames);
      }
      rigidBodyManagers.add(headManager);

      // Create hand managers
      for (RobotSide side : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(side);
         if (hand != null)
         {
            ReferenceFrame handFrame = hand.getBodyFixedFrame();
            RigidBodyControlManager handManager = motionControlManagerFactory.getOrCreateRigidBodyManager(hand, chest, handFrame, chestFrame, trajectoryFrames);
            handManagers.put(side, handManager);
            rigidBodyManagers.add(handManager);
         }
      }

      // Create foot control managers
      for (RobotSide side : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(side);
         ContactableFoot contactableFoot = controllerToolbox.getContactableFeet().get(side);
         FootController footController = new FootController(side.getCamelCaseNameForStartOfExpression() + "Foot", controllerToolbox.getYoTime(), contactState,
                                                            contactableFoot, fullRobotModel.getRootBody(), pelvis, registry);
         feetControllers.put(side, footController);
      }
   }

   private StateMachine<MotionControllerStateEnum, MotionControllerState> setupStateMachine()
   {
      StateMachineFactory<MotionControllerStateEnum, MotionControllerState> factory = new StateMachineFactory<>(MotionControllerStateEnum.class);
      factory.setNamePrefix("jumpController").setRegistry(registry).buildYoClock(yoTime);
      DoubleSupportMotionState doubleSupportState = new DoubleSupportMotionState(requestedState, chestManager, headManager, handManagers, feetControllers,
                                                                                 registry);
      factory.addState(MotionControllerStateEnum.DOUBLE_SUPPORT, doubleSupportState);
      return factory.build(MotionControllerStateEnum.DOUBLE_SUPPORT);
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   private void submitControllerCommands()
   {
      planeContactStateCommandPool.clear();
      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);

      for (int i = 0; i < controlManagerList.size(); i++)
      {
         ControlManagerInterface controlManager = controlManagerList.get(i);
         controllerCoreCommand.addInverseDynamicsCommand(controlManager.getInverseDynamicsCommand());
         controllerCoreCommand.addFeedbackControlCommand(controlManager.getFeedbackControlCommand());
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         planeContactStateCommand.setUseHighCoPDamping(false);
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
      }

      for (int i = 0; i < rigidBodyManagers.size(); i++)
      {
         RigidBodyControlManager manager = rigidBodyManagers.get(i);
         if (manager != null)
         {
            controllerCoreCommand.addInverseDynamicsCommand(manager.getInverseDynamicsCommand());
            controllerCoreCommand.addFeedbackControlCommand(manager.getFeedbackControlCommand());
         }
      }
      controllerCoreCommand.addInverseDynamicsCommand(createZeroAccelerationMomentumCommand());
   }

   // Temp momentum method for debugging
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final FrameVector3D zeroVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);

   private InverseDynamicsCommand<?> createZeroAccelerationMomentumCommand()
   {
      momentumRateCommand.setLinearMomentumRate(zeroVector);
      momentumRateCommand.setAngularMomentumRate(zeroVector);
      momentumRateCommand.setAngularWeights(controllerParamaters.getMomentumOptimizationSettings().getAngularMomentumWeight());
      momentumRateCommand.setLinearWeights(controllerParamaters.getMomentumOptimizationSettings().getLinearMomentumWeight());
      momentumRateCommand.setSelectionMatrixToIdentity();
      return momentumRateCommand;
   }
   // Ends here

   private void initializeManagers()
   {
      for (int i = 0; i < rigidBodyManagers.size(); i++)
         rigidBodyManagers.get(i).initialize();
      for (int i = 0; i < controlManagerList.size(); i++)
         controlManagerList.get(i).initialize();
   }

   @Override
   public void initialize()
   {
      controllerCoreCommand.requestReinitialization();
      controllerToolbox.initialize();
      initializePriviledgedConfiguration();
      initializeManagers();
   }

   private void initializePriviledgedConfiguration()
   {
      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getArmJoint(robotSide, armJointNames[i]), PrivilegedConfigurationOption.AT_ZERO);

         LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
         for (int i = 0; i < legJointNames.length; i++)
            privilegedConfigurationCommand.addJoint(fullRobotModel.getLegJoint(robotSide, legJointNames[i]), PrivilegedConfigurationOption.AT_ZERO);
      }
   }

   @Override
   public void doAction()
   {
      stateMachine.doActionAndTransition();
      submitControllerCommands();
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
