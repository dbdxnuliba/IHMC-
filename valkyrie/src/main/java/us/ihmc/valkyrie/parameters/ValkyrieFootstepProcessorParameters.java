package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.networkProcessor.footstepProcessingToolboxModule.FootstepProcessingParameters;

public class ValkyrieFootstepProcessorParameters implements FootstepProcessingParameters
{
   @Override
   public double getMinimumSwingTime()
   {
      return 1.2;
   }

   @Override
   public double getMaximumSwingTime()
   {
      return 2.4;
   }

   @Override
   public double getMaximumStepTranslationForMinimumSwingTime()
   {
      return 0.2;
   }

   @Override
   public double getMinimumStepTranslationForMaximumSwingTime()
   {
      return 0.6;
   }
}
