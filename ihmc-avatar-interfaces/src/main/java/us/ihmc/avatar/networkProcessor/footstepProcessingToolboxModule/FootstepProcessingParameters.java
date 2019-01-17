package us.ihmc.avatar.networkProcessor.footstepProcessingToolboxModule;

/**
 * Parameters for setting swing times and trajectories from a footstep plan.
 */
public interface FootstepProcessingParameters
{
   double getMinimumSwingTime();

   double getMaximumSwingTime();

   double getMaximumStepTranslationForMinimumSwingTime();

   double getMinimumStepTranslationForMaximumSwingTime();
}
