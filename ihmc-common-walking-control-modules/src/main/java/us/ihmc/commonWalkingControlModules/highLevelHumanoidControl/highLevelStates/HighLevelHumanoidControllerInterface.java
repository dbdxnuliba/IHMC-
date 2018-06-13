package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;

public interface HighLevelHumanoidControllerInterface
{
   void doAction();
   
   ControllerCoreCommand getControllerCoreCommand();
   
   void initialize();
}
