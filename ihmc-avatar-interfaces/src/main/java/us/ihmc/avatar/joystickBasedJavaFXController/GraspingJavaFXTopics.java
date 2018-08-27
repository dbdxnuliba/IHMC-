package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Category;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TypedTopicTheme;

public class GraspingJavaFXTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("Grasping"));

   private static final CategoryTheme GraspingObject = apiFactory.createCategoryTheme("GraspingObject");
   private static final CategoryTheme FingerControl = apiFactory.createCategoryTheme("FingerControl");

   private static final CategoryTheme Sphere = apiFactory.createCategoryTheme("Sphere");
   private static final CategoryTheme Cylinder = apiFactory.createCategoryTheme("Cylinder");
   private static final CategoryTheme Torus = apiFactory.createCategoryTheme("Torus");
   private static final CategoryTheme Box = apiFactory.createCategoryTheme("Box");
   private static final CategoryTheme Left = apiFactory.createCategoryTheme("Left");
   private static final CategoryTheme Right = apiFactory.createCategoryTheme("Right");
   private static final CategoryTheme Thumb = apiFactory.createCategoryTheme("Thumb");
   private static final CategoryTheme Index = apiFactory.createCategoryTheme("Index");
   private static final CategoryTheme Middle = apiFactory.createCategoryTheme("Middle");
   private static final CategoryTheme Pinky = apiFactory.createCategoryTheme("Pinky");

   private static final TypedTopicTheme<Double> Radius = apiFactory.createTypedTopicTheme("Radius");
   private static final TypedTopicTheme<Double> Height = apiFactory.createTypedTopicTheme("Height");
   private static final TypedTopicTheme<Double> TubeRadius = apiFactory.createTypedTopicTheme("TubeRadius");
   private static final TypedTopicTheme<Double> Length = apiFactory.createTypedTopicTheme("Length");
   private static final TypedTopicTheme<Double> Width = apiFactory.createTypedTopicTheme("Width");
   private static final TypedTopicTheme<Double> Roll = apiFactory.createTypedTopicTheme("Roll");
   private static final TypedTopicTheme<Double> Pitch = apiFactory.createTypedTopicTheme("Pitch");
   private static final TypedTopicTheme<Double> Pitch2 = apiFactory.createTypedTopicTheme("Pitch2");
   private static final TypedTopicTheme<Boolean> SendMessage = apiFactory.createTypedTopicTheme("SendMessage");

   private static final TypedTopicTheme<Integer> Selected = apiFactory.createTypedTopicTheme("Selected");

   public static final Topic<Double> SphereRadius = Root.child(GraspingObject).child(Sphere).topic(Radius);
   public static final Topic<Double> CylinderRadius = Root.child(GraspingObject).child(Cylinder).topic(Radius);
   public static final Topic<Double> CylinderHeight = Root.child(GraspingObject).child(Cylinder).topic(Height);
   public static final Topic<Double> TorusRadius = Root.child(GraspingObject).child(Torus).topic(Radius);
   public static final Topic<Double> TorusTubeRadius = Root.child(GraspingObject).child(Torus).topic(TubeRadius);
   public static final Topic<Double> BoxLength = Root.child(GraspingObject).child(Box).topic(Length);
   public static final Topic<Double> BoxWidth = Root.child(GraspingObject).child(Box).topic(Width);
   public static final Topic<Double> BoxHeight = Root.child(GraspingObject).child(Box).topic(Height);
   public static final Topic<Double> LeftThumbRoll = Root.child(FingerControl).child(Left).child(Thumb).topic(Roll);
   public static final Topic<Double> LeftThumb = Root.child(FingerControl).child(Left).child(Thumb).topic(Pitch);
   public static final Topic<Double> LeftThumb2 = Root.child(FingerControl).child(Left).child(Thumb).topic(Pitch2);
   public static final Topic<Double> LeftIndex = Root.child(FingerControl).child(Left).child(Index).topic(Pitch);
   public static final Topic<Double> LeftMiddle = Root.child(FingerControl).child(Left).child(Middle).topic(Pitch);
   public static final Topic<Double> LeftPinky = Root.child(FingerControl).child(Left).child(Pinky).topic(Pitch);
   public static final Topic<Double> RightThumbRoll = Root.child(FingerControl).child(Right).child(Thumb).topic(Roll);
   public static final Topic<Double> RightThumb = Root.child(FingerControl).child(Right).child(Thumb).topic(Pitch);
   public static final Topic<Double> RightThumb2 = Root.child(FingerControl).child(Right).child(Thumb).topic(Pitch2);
   public static final Topic<Double> RightIndex = Root.child(FingerControl).child(Right).child(Index).topic(Pitch);
   public static final Topic<Double> RightMiddle = Root.child(FingerControl).child(Right).child(Middle).topic(Pitch);
   public static final Topic<Double> RightPinky = Root.child(FingerControl).child(Right).child(Pinky).topic(Pitch);
   public static final Topic<Boolean> LeftSendMessage = Root.child(FingerControl).child(Left).topic(SendMessage);
   public static final Topic<Boolean> RightSendMessage = Root.child(FingerControl).child(Right).topic(SendMessage);

   public static final Topic<Integer> SelectedShape = Root.child(GraspingObject).topic(Selected);

   static
   {
      apiFactory.includeMessagerAPIs(XBoxOneJavaFXController.XBoxOneControllerAPI);
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}