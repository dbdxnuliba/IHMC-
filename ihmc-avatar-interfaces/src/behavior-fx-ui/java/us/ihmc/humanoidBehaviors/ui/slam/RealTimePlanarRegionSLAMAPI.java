package us.ihmc.humanoidBehaviors.ui.slam;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class RealTimePlanarRegionSLAMAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme PlanarRegionUpdater = apiFactory.createCategoryTheme("PlanarRegionUpdater");
   private static final CategoryTheme SLAM = apiFactory.createCategoryTheme("SLAM");

   private static final CategoryTheme Result = apiFactory.createCategoryTheme("Result");
   private static final CategoryTheme Map = apiFactory.createCategoryTheme("Map");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<RigidBodyTransform> Transform = apiFactory.createTypedTopicTheme("Transform");
   private static final TypedTopicTheme<String> Status = apiFactory.createTypedTopicTheme("Status");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("RealTimeSLAM"));

   public static final Topic<Boolean> EnablePlanarRegionIncoming = Root.child(PlanarRegionUpdater).topic(Enable);
   public static final Topic<Boolean> ClearPlanarRegion = Root.child(PlanarRegionUpdater).topic(Clear);
   public static final Topic<String> PlanarRegionStatus = Root.child(PlanarRegionUpdater).topic(Status);

   public static final Topic<Boolean> EnableSLAM = Root.child(SLAM).topic(Enable);
   public static final Topic<Boolean> ShowSLAMResult = Root.child(SLAM).child(Result).topic(Show);
   public static final Topic<RigidBodyTransform> LatestSLAMTransform = Root.child(SLAM).child(Result).topic(Transform);
   public static final Topic<Boolean> ShowSLAMMap = Root.child(SLAM).child(Map).topic(Show);
   public static final Topic<Boolean> ClearSLAMMap = Root.child(SLAM).topic(Clear);

   public static final Topic<PlanarRegionsList> PlanarRegionsList = Root.child(PlanarRegionUpdater).topic(Data);
   public static final Topic<Boolean> SLAMDummyParameter = Root.child(SLAM).topic(Parameters); //TODO: replace Boolean to *Parameters

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
