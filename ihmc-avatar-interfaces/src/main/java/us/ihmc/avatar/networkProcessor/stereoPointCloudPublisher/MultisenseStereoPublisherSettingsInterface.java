package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import us.ihmc.euclid.tuple2D.Point2D;

public interface MultisenseStereoPublisherSettingsInterface
{
   public boolean useVelocityFilter();

   public double getLinearVelocityThreshold();

   public double getAngularVelocityThreshold();

   public boolean useIntensiveBox();

   public String getTopicNameForIntensiveBox();

   public Point2D getIntensiveBoxUpperPoint();

   public Point2D getIntensiveBoxLowerPoint();
}
