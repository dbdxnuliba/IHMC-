package us.ihmc.quadrupedBasics.heightMap;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class HeightMapEstimatorMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");

   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("HeightMapEstimator"));

   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(PlanarRegion).topic(Data);
   public static final Topic<Boolean> ShowPlanarRegionsTopic = Root.child(PlanarRegion).topic(Show);
   public static final Topic<Point3D> PointToAddTopic = Root.child(PlanarRegion).topic(Position);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}