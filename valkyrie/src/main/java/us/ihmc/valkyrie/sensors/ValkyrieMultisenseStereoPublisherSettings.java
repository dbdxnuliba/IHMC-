package us.ihmc.valkyrie.sensors;

import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.MultisenseStereoPublisherSettingsInterface;

public class ValkyrieMultisenseStereoPublisherSettings implements MultisenseStereoPublisherSettingsInterface
{
   @Override
   public boolean useVelocityFilter()
   {
      return false;
   }

   @Override
   public boolean useIntensiveBox()
   {
      return false;
   }
}