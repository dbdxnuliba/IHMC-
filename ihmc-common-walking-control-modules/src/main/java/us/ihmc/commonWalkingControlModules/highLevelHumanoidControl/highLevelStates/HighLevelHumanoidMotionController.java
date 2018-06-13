package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerParamaters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states.MotionControllerStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class HighLevelHumanoidMotionController implements HighLevelHumanoidControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // Decision variables and parameters
   private final GenericStateMachine<MotionControllerStateEnum, MotionControllerState> stateMachine;
   private final MotionControllerParamaters controllerParamaters;

   // IO objects
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private ControllerCoreOutputReadOnly controllerCoreOutput;

   public HighLevelHumanoidMotionController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                            HighLevelHumanoidControllerToolbox controllerToolbox, MotionControllerParamaters motionControllerParameters,
                                            YoVariableRegistry parentRegistry)
   {
      controllerParamaters = motionControllerParameters;
      String name = "motionController";
      stateMachine = new GenericStateMachine<>(name + "State", name + "SwitchTime", MotionControllerStateEnum.class, controllerToolbox.getYoTime(), registry);
      parentRegistry.addChild(registry);
   }

   private void stupStateMachine()
   {

   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void doAction()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }
}
