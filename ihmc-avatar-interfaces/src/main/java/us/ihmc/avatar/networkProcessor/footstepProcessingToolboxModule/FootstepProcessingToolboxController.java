package us.ihmc.avatar.networkProcessor.footstepProcessingToolboxModule;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootstepProcessingToolboxController extends ToolboxController
{
   public FootstepProcessingToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
   }

   @Override
   public boolean initialize()
   {
      return false;
   }

   @Override
   public void updateInternal() throws Exception
   {

   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
