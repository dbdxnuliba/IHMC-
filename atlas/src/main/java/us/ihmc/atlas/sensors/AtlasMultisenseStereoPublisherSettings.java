package us.ihmc.atlas.sensors;

import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.MultisenseStereoPublisherSettingsInterface;
import us.ihmc.euclid.tuple2D.Point2D;

public class AtlasMultisenseStereoPublisherSettings implements MultisenseStereoPublisherSettingsInterface
{
   @Override
   public boolean useVelocityFilter()
   {
      return false;
   }

   @Override
   public double getLinearVelocityThreshold()
   {
      return AtlasSensorInformation.linearVelocityThreshold;
   }

   @Override
   public double getAngularVelocityThreshold()
   {
      return AtlasSensorInformation.angularVelocityThreshold;
   }

   @Override
   public boolean useIntensiveBox()
   {
      return true;
   }

   @Override
   public String getTopicNameForIntensiveBox()
   {
      return AtlasSensorInformation.stereoColorTopicLocal;
   }

   @Override
   public Point2D getIntensiveBoxUpperPoint()
   {
      return new Point2D(0.5, 0.5);
   }

   @Override
   public Point2D getIntensiveBoxLowerPoint()
   {
      return new Point2D(-0.5, -0.5);
   }
}
