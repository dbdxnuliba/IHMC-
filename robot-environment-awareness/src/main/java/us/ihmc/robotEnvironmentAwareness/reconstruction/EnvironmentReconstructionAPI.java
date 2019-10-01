package us.ihmc.robotEnvironmentAwareness.reconstruction;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;

public class EnvironmentReconstructionAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");

   private static final CategoryTheme Source = apiFactory.createCategoryTheme("Source");
   private static final CategoryTheme LidarScan = apiFactory.createCategoryTheme("LidarScan");
   private static final CategoryTheme StereoPointCloud = apiFactory.createCategoryTheme("StereoPointCloud");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("REA"));

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   public static final Topic<LidarScanMessage> LidarScanState = Root.child(Source).child(LidarScan).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> StereoVisionPointCloudState = Root.child(Source).child(StereoPointCloud).topic(Data);

   public static final Topic<Boolean> LidarScanEnable = Root.child(Source).child(LidarScan).topic(Enable);
   public static final Topic<Boolean> StereoPointCloudEnable = Root.child(Source).child(StereoPointCloud).topic(Enable);
   public static final Topic<Boolean> LidarScanClear = Root.child(Source).child(LidarScan).topic(Clear);
   public static final Topic<Boolean> StereoPointCloudClear = Root.child(Source).child(StereoPointCloud).topic(Clear);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
