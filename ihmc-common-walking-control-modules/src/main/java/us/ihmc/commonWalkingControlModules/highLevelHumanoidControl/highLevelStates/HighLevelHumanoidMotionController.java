package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.DoubleSupportMotionState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerParamaters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class HighLevelHumanoidMotionController implements HighLevelHumanoidControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final MotionControllerParamaters controllerParamaters;

   // Control modules and such
   private final GenericStateMachine<MotionControllerStateEnum, MotionControllerState> stateMachine;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final FullHumanoidRobotModel fullRobotModel;
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final List<ControlManagerInterface> controlManagerList = new ArrayList<>();
   private final List<RigidBodyControlManager> rigidBodyManagers = new ArrayList<>();
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(PlaneContactStateCommand.class);

   // IO objects
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   public HighLevelHumanoidMotionController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                            HighLevelHumanoidControllerToolbox controllerToolbox, MotionControllerParamaters motionControllerParameters,
                                            YoVariableRegistry parentRegistry)
   {
      controllerParamaters = motionControllerParameters;
      String name = "motionController";
      this.controllerToolbox = controllerToolbox;
      fullRobotModel = controllerToolbox.getFullRobotModel();
      stateMachine = new GenericStateMachine<>(name + "State", name + "SwitchTime", MotionControllerStateEnum.class, controllerToolbox.getYoTime(), registry);
      setupStateMachine();
      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      DoubleSupportMotionState doubleSupportState = new DoubleSupportMotionState();
      stateMachine.addState(doubleSupportState);
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
   }

   private void initializeManagers()
   {

   }

   @Override
   public void initialize()
   {
      stateMachine.setCurrentState(MotionControllerStateEnum.DOUBLE_SUPPORT);
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
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      submitControllerCommands();
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
