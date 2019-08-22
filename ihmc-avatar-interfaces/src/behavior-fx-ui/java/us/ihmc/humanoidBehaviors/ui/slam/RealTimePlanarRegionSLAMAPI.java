package us.ihmc.humanoidBehaviors.ui.slam;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;

public class RealTimePlanarRegionSLAMAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme PlanarRegionUpdater = apiFactory.createCategoryTheme("PlanarRegionUpdater");
   private static final CategoryTheme SLAM = apiFactory.createCategoryTheme("SLAM");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("RealTimeSLAM"));

   public static final Topic<Boolean> PlanarRegionIncomingEnable = Root.child(PlanarRegionUpdater).topic(Enable);
   public static final Topic<Boolean> SLAMEnable = Root.child(SLAM).topic(Enable);

   public static final Topic<Boolean> SLAMDummyParameter = Root.child(SLAM).topic(Parameters); //TODO: replace Boolean to *Parameters

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
