package us.ihmc.robotEnvironmentAwareness.fusion;

import org.jcodec.common.IntArrayList;

import controller_msgs.msg.dds.ImageMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;

public class LidarImageFusionAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("LidarImageFusion"));
   
   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UserInterface");
   
   private static final Category ModuleCategory = Root.child(Module);
   private static final Category UICategory = ModuleCategory.child(UI);
   
   private static final CategoryTheme Image = apiFactory.createCategoryTheme("Image");
   private static final CategoryTheme ObjectDetection = apiFactory.createCategoryTheme("ObjectDetection");
   private static final CategoryTheme Door = apiFactory.createCategoryTheme("Door");
   
   private static final TypedTopicTheme<Boolean> SnapShot = apiFactory.createTypedTopicTheme("SnapShot");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Detect = apiFactory.createTypedTopicTheme("Detect");
   
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme ROI = apiFactory.createTopicTheme("ROI");
   
   public static final Topic<Boolean> ImageSnapShot = UICategory.child(Image).topic(SnapShot);
   public static final Topic<Boolean> EnableStreaming = UICategory.child(Image).topic(Enable);
   public static final Topic<Boolean> DetectDoor = UICategory.child(ObjectDetection).child(Door).topic(Detect);
   
   public static final Topic<IntArrayList> doorROI = ModuleCategory.child(ObjectDetection).child(Door).topic(ROI);
   
   public static final Topic<ImageMessage> ImageState = ModuleCategory.child(Image).topic(Data);
   
   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
