package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import us.ihmc.euclid.tuple2D.Point2D;

public interface MultisenseStereoPublisherSettingsInterface
{
   public boolean useVelocityFilter();
   
   default double getLinearVelocityThreshold()
   {
      return 0.0;
   }
   
   default double getAngularVelocityThreshold()
   {
      return 0.0;
   }
   
   public boolean useIntensiveBox();
   
   default String getTopicNameForIntensiveBox()
   {
      return "";
   }

   default Point2D getIntensiveBoxUpperPoint()
   {
      return null;
   }

   default Point2D getIntensiveBoxLowerPoint()
   {
      return null;
   }
}
