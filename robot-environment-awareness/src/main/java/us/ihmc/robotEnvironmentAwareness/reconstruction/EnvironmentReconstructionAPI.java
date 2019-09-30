package us.ihmc.robotEnvironmentAwareness.reconstruction;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;

public class EnvironmentReconstructionAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UserInterface");
   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");

   private static final CategoryTheme SensorOrigin = apiFactory.createCategoryTheme("SensorOrigin");
   private static final CategoryTheme Source = apiFactory.createCategoryTheme("Source");
   private static final CategoryTheme Filter = apiFactory.createCategoryTheme("Filter");
   private static final CategoryTheme StereoPointCloud = apiFactory.createCategoryTheme("StereoPointCloud");
   private static final CategoryTheme Octree = apiFactory.createCategoryTheme("Octree");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("REA"));
   private static final Category UICategry = Root.child(UI);
   private static final Category SensorOriginCategry = Root.child(Module).child(SensorOrigin);
   private static final Category SourceCategry = Root.child(Module).child(Source);
   private static final Category FilterCategory = Root.child(Module).child(Filter);

   public static final Topic<Boolean> SensorOriginEnable = SourceCategry.child(SensorOrigin).topic(Enable);
   public static final Topic<Boolean> StereoPointCloudEnable = SourceCategry.child(StereoPointCloud).topic(Enable);
   public static final Topic<Boolean> OctreeEnable = SourceCategry.child(Octree).topic(Enable);
   public static final Topic<Boolean> PlanarRegionsEnable = SourceCategry.child(PlanarRegions).topic(Enable);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
