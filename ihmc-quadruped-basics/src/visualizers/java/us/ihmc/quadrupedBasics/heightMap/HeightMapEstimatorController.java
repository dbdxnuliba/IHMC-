package us.ihmc.quadrupedBasics.heightMap;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.quadrupedBasics.heightMap.HeightMapEstimator;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class HeightMapEstimatorController
{
   private JavaFXMessager messager;

   private final HeightMapEstimator heightMapEstimator = new HeightMapEstimator(null);

   public void attachMessager(JavaFXMessager messager, Topic<Point3D> pointToAddTopic, Topic<PlanarRegionsList> planarRegionDataTopic)
   {
      this.messager = messager;

      messager.registerTopicListener(pointToAddTopic, heightMapEstimator::processIncomingPoint);

      heightMapEstimator.attachHeightMapListener(heightMap -> messager.submitMessage(planarRegionDataTopic, heightMap));
   }
}
