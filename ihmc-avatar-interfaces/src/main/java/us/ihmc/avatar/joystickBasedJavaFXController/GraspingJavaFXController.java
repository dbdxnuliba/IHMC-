package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.animation.AnimationTimer;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.avatar.handControl.HandFingerTrajectoryMessagePublisher;
import us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule.WholeBodyTrajectoryToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.euclid.geometry.Sphere3D;
import us.ihmc.euclid.geometry.Torus3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.WholeBodyTrajectoryToolboxOutputConverter;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.manipulation.planning.manifold.ReachingManifoldTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.ros2.Ros2Node;

/**
 * What to do this.
 * Handler grasping object.
 * Communicate with WholeBodyTrajectoryToolbox.
 * Control fingers.
 * Visualize object and preview of motion.
 */

public class GraspingJavaFXController
{
   private final JavaFXMessager messager;
   private final FullHumanoidRobotModel fullRobotModel;

   private final WholeBodyTrajectoryToolboxOutputConverter outputConverter;

   private final AtomicReference<List<Node>> objectsToVisualizeReference = new AtomicReference<>(new ArrayList<>());

   private final AtomicReference<Double> sphereRadius;

   private final AtomicReference<Double> cylinderRadius;
   private final AtomicReference<Double> cylinderHeight;

   private final AtomicReference<Double> torusRadius;
   private final AtomicReference<Double> torusTube;

   private final AtomicReference<Double> boxLength;
   private final AtomicReference<Double> boxWidth;
   private final AtomicReference<Double> boxHeight;

   private final SideDependentList<AtomicReference<Boolean>> sendFingerMessages = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbRolls = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbPitchs = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbPitch2s = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredIndexes = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredMiddles = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredPinkys = new SideDependentList<>();

   private final AnimationTimer animationTimer;

   private final Group rootNode = new Group();

   private final static Vector3D defaultPositionToCreateObject = new Vector3D(0.6, 0.3, 1.0);
   private final Point3D controlPosition = new Point3D(defaultPositionToCreateObject);
   private final RotationMatrix controlOrientation = new RotationMatrix();
   private final FramePoint3D controlFramePosition;
   private final FrameQuaternion controlFrameOrientation;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame pelvisZUpFrame;

   private boolean toolboxSelectingSide = false;
   private RobotSide reachingSide = null;
   private final static double timeDurationForFinger = 2.0;

   private final static double ratioJoyStickToPosition = 0.01;
   private final static double ratioJoyStickToRotation = 0.02;

   private final static double lengthOfControlFrame = 0.2;

   private final DoubleProperty velocityXProperty = new SimpleDoubleProperty(this, "velocityXProperty", 0.0);
   private final DoubleProperty velocityYProperty = new SimpleDoubleProperty(this, "velocityYProperty", 0.0);
   private final DoubleProperty velocityZProperty = new SimpleDoubleProperty(this, "velocityZProperty", 0.0);
   private final DoubleProperty velocityRollProperty = new SimpleDoubleProperty(this, "velocityRollProperty", 0.0);
   private final DoubleProperty velocityPitchProperty = new SimpleDoubleProperty(this, "velocityPitchProperty", 0.0);
   private final DoubleProperty velocityYawProperty = new SimpleDoubleProperty(this, "velocityYawProperty", 0.0);

   private ShapeToCreate shapeToCreate = null;
   private int indexOfSelectedObject = 0;
   private final List<Shape3D<?>> listOfShape3D = new ArrayList<>();

   private final IHMCROS2Publisher<WholeBodyTrajectoryMessage> wholeBodyTrajectoryPublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<WholeBodyTrajectoryToolboxMessage> toolboxMessagePublisher;
   private final HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher;

   private final AtomicReference<WholeBodyTrajectoryToolboxOutputStatus> toolboxOutputPacket = new AtomicReference<>(null);
   private final GraspingJavaFXMotionPreviewVisualizer motionPreviewVisualizer;

   private enum ShapeToCreate
   {
      Sphere, Cylinder, Torus, Box;
      public ShapeToCreate nextShape()
      {
         int nextOrdinal = this.ordinal() + 1;
         for (ShapeToCreate shape : ShapeToCreate.values())
            if (shape.ordinal() == nextOrdinal)
               return shape;
         return Sphere;
      }
   }

   public GraspingJavaFXController(String robotName, JavaFXMessager messager, Ros2Node ros2Node, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                   JavaFXRobotVisualizer javaFXRobotVisualizer, HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher)
   {
      this.messager = messager;
      motionPreviewVisualizer = new GraspingJavaFXMotionPreviewVisualizer(fullRobotModelFactory);

      fullRobotModel = javaFXRobotVisualizer.getFullRobotModel();
      outputConverter = new WholeBodyTrajectoryToolboxOutputConverter(fullRobotModelFactory);

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      controlFramePosition = new FramePoint3D(pelvisZUpFrame, defaultPositionToCreateObject);
      controlFrameOrientation = new FrameQuaternion(pelvisZUpFrame, new Quaternion());

      sphereRadius = messager.createInput(GraspingJavaFXTopics.SphereRadius, 0.1);

      cylinderRadius = messager.createInput(GraspingJavaFXTopics.CylinderRadius, 0.1);
      cylinderHeight = messager.createInput(GraspingJavaFXTopics.CylinderHeight, 0.1);

      torusRadius = messager.createInput(GraspingJavaFXTopics.TorusRadius, 0.1);
      torusTube = messager.createInput(GraspingJavaFXTopics.TorusTubeRadius, 0.1);

      boxLength = messager.createInput(GraspingJavaFXTopics.BoxLength);
      boxWidth = messager.createInput(GraspingJavaFXTopics.BoxWidth);
      boxHeight = messager.createInput(GraspingJavaFXTopics.BoxHeight);

      sendFingerMessages.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightSendMessage, false));
      sendFingerMessages.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftSendMessage, false));

      desiredThumbRolls.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightThumbRoll, 0.0));
      desiredThumbPitchs.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightThumb, 0.0));
      desiredThumbPitch2s.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightThumb2, 0.0));
      desiredIndexes.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightIndex, 0.0));
      desiredMiddles.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightMiddle, 0.0));
      desiredPinkys.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightPinky, 0.0));

      desiredThumbRolls.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftThumbRoll, 0.0));
      desiredThumbPitchs.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftThumb, 0.0));
      desiredThumbPitch2s.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftThumb2, 0.0));
      desiredIndexes.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftIndex, 0.0));
      desiredMiddles.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftMiddle, 0.0));
      desiredPinkys.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftPinky, 0.0));

      messager.registerTopicListener(XBoxOneJavaFXController.ButtonSelectState, state -> clearObjects(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonStartState, state -> switchShapeToCreate(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonAState, state -> createObject(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonBState, state -> switchSelectedObject(state));

      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftStickYAxis, this::appendingXAxis);
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftStickXAxis, this::appendingYAxis);
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonLeftBumperState, state -> appendingZAxisPositive(state));
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftTriggerAxis, this::appendingZAxisNegative);

      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightStickYAxis, this::appendingPitch);
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightStickXAxis, this::appendingRoll);
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonRightBumperState, state -> appendingYawPositive(state));
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightTriggerAxis, this::appendingYawNegative);

      messager.registerTopicListener(XBoxOneJavaFXController.ButtonXState, state -> sendReachingManifoldsToToolbox(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonYState, state -> confirmReachingMotion(state));

      ROS2Tools.MessageTopicNameGenerator toolboxRequestTopicNameGenerator = WholeBodyTrajectoryToolboxModule.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator toolboxResponseTopicNameGenerator = WholeBodyTrajectoryToolboxModule.getPublisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      wholeBodyTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, WholeBodyTrajectoryMessage.class, subscriberTopicNameGenerator);
      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, toolboxRequestTopicNameGenerator);
      toolboxMessagePublisher = ROS2Tools.createPublisher(ros2Node, WholeBodyTrajectoryToolboxMessage.class, toolboxRequestTopicNameGenerator);
      this.handFingerTrajectoryMessagePublisher = handFingerTrajectoryMessagePublisher;

      ROS2Tools.createCallbackSubscription(ros2Node, WholeBodyTrajectoryToolboxOutputStatus.class, toolboxResponseTopicNameGenerator,
                                           s -> consumeToolboxOutputStatus(s.takeNextData()));

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long arg0)
         {
            updateSelectedObject();
            updateVisualizationObjects();
            rootNode.getChildren().add(motionPreviewVisualizer.getRootNode());
            submitDesiredFingerConfigurationMessage();
         }
      };
   }

   private void submitDesiredFingerConfigurationMessage()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (sendFingerMessages.get(robotSide).get())
         {
            sendFingerMessages.get(robotSide).set(false);

            TDoubleArrayList desiredPositions = new TDoubleArrayList();
            TDoubleArrayList trajectoryTimes = new TDoubleArrayList();

            desiredPositions.add(desiredThumbRolls.get(robotSide).get());
            desiredPositions.add(desiredThumbPitchs.get(robotSide).get());
            desiredPositions.add(desiredThumbPitch2s.get(robotSide).get());
            desiredPositions.add(desiredIndexes.get(robotSide).get());
            desiredPositions.add(desiredMiddles.get(robotSide).get());
            desiredPositions.add(desiredPinkys.get(robotSide).get());

            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);

            handFingerTrajectoryMessagePublisher.sendFingerTrajectoryMessage(robotSide, desiredPositions, trajectoryTimes);
         }
      }
   }

   private void consumeToolboxOutputStatus(WholeBodyTrajectoryToolboxOutputStatus packet)
   {
      PrintTools.info("packet arrived");
      toolboxOutputPacket.set(packet);
      PrintTools.info("" + toolboxOutputPacket.get().getPlanningResult());
      PrintTools.info("" + toolboxOutputPacket.get().getRobotConfigurations());

      if (toolboxOutputPacket.get().getPlanningResult() == 4)
      {
         motionPreviewVisualizer.enable(true);
         motionPreviewVisualizer.submitWholeBodyTrajectoryToolboxOutputStatus(toolboxOutputPacket.get());
      }
   }

   private void sendReachingManifoldsToToolbox(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
         toolboxSelectingSide = true;
      else
         toolboxSelectingSide = false;

      if (state == ButtonState.RELEASED)
      {
         if (reachingSide != null)
         {
            if (listOfShape3D.size() > 0)
            {
               motionPreviewVisualizer.enable(false);
               toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

               RobotSide robotSide = reachingSide;
               RigidBody hand = fullRobotModel.getHand(robotSide);

               List<ReachingManifoldMessage> reachingManifoldMessages = new ArrayList<>();
               for (int i = 0; i < listOfShape3D.size(); i++)
               {
                  List<ReachingManifoldMessage> manifolds = null;
                  Shape3D<?> shape3d = listOfShape3D.get(i);
                  if (shape3d instanceof Sphere3D)
                  {
                     manifolds = ReachingManifoldTools.createSphereManifoldMessagesForValkyrie(robotSide, hand, shape3d.getPosition(),
                                                                                               ((Sphere3D) shape3d).getRadius());
                  }
                  else if (shape3d instanceof Cylinder3D)
                  {
                     manifolds = ReachingManifoldTools.createCylinderManifoldMessagesForValkyrie(robotSide, hand, shape3d.getPosition(),
                                                                                                 shape3d.getOrientation(), ((Cylinder3D) shape3d).getRadius(),
                                                                                                 ((Cylinder3D) shape3d).getHeight());
                  }
                  else if (shape3d instanceof Torus3D)
                  {
                     manifolds = ReachingManifoldTools.createTorusManifoldMessagesForValkyrie(robotSide, hand, shape3d.getPosition(), shape3d.getOrientation(),
                                                                                              ((Torus3D) shape3d).getRadius(),
                                                                                              ((Torus3D) shape3d).getTubeRadius());
                  }

                  if (manifolds != null)
                     reachingManifoldMessages.addAll(manifolds);
                  else
                  {
                     System.out.println("there is no objects created");
                     break;
                  }
               }

               System.out.println("number of manifolds " + reachingManifoldMessages.size());
               System.out.println("reaching side is " + reachingSide);
               WholeBodyTrajectoryToolboxMessage wbtmessage = ReachingManifoldTools.createReachingWholeBodyTrajectoryToolboxMessage(fullRobotModel, robotSide,
                                                                                                                                    reachingManifoldMessages,
                                                                                                                                    5.0);

               toolboxMessagePublisher.publish(wbtmessage);

               reachingSide = null;
            }
            else
               System.out.println("there is no objects created");
         }
         else
            System.out.println("robot side is not selected");
      }
   }

   private void confirmReachingMotion(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         PrintTools.info("confirmReachingMotion");

         motionPreviewVisualizer.enable(false);

         WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();

         outputConverter.setMessageToCreate(message);
         outputConverter.updateFullRobotModel(toolboxOutputPacket.get());

         wholeBodyTrajectoryPublisher.publish(message);
      }
   }

   private void updateSelectedObject()
   {
      pelvisZUpFrame.update();
      controlFramePosition.changeFrame(pelvisZUpFrame);
      controlFrameOrientation.changeFrame(pelvisZUpFrame);

      controlFramePosition.add(velocityXProperty.getValue(), velocityYProperty.getValue(), velocityZProperty.getValue());
      controlFrameOrientation.prependRollRotation(velocityRollProperty.getValue());
      controlFrameOrientation.prependPitchRotation(velocityPitchProperty.getValue());
      controlFrameOrientation.prependYawRotation(velocityYawProperty.getValue());

      controlFramePosition.changeFrame(worldFrame);
      controlFrameOrientation.changeFrame(worldFrame);

      controlPosition.set(controlFramePosition);
      controlOrientation.set(controlFrameOrientation);
   }

   private void appendingRoll(double alpha)
   {
      velocityRollProperty.set(-alpha * ratioJoyStickToRotation);
   }

   private void appendingPitch(double alpha)
   {
      velocityPitchProperty.set(alpha * ratioJoyStickToRotation);
   }

   private void appendingYawNegative(double alpha)
   {
      velocityYawProperty.set(alpha * ratioJoyStickToRotation);
   }

   private void appendingYawPositive(ButtonState state)
   {
      if (toolboxSelectRightSide(state))
         ;
      else
      {
         if (state == ButtonState.PRESSED)
            velocityYawProperty.set(ratioJoyStickToRotation);
         else
            velocityYawProperty.set(0.0);
      }
   }

   private void appendingXAxis(double alpha)
   {
      velocityXProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingYAxis(double alpha)
   {
      velocityYProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingZAxisNegative(double alpha)
   {
      velocityZProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingZAxisPositive(ButtonState state)
   {
      if (toolboxSelectLeftSide(state))
         ;
      else
      {
         if (state == ButtonState.PRESSED)
            velocityZProperty.set(ratioJoyStickToPosition);
         else
            velocityZProperty.set(0.0);
      }
   }

   private boolean toolboxSelectRightSide(ButtonState state)
   {
      if (toolboxSelectingSide)
      {
         if (state == ButtonState.PRESSED)
            reachingSide = RobotSide.RIGHT;

         return true;
      }
      else
         return false;
   }

   private boolean toolboxSelectLeftSide(ButtonState state)
   {
      if (toolboxSelectingSide)
      {
         if (state == ButtonState.PRESSED)
            reachingSide = RobotSide.LEFT;

         return true;
      }
      else
         return false;
   }

   private void switchShapeToCreate(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         if (shapeToCreate == null)
            shapeToCreate = ShapeToCreate.Sphere;
         else
            shapeToCreate = shapeToCreate.nextShape();

         messager.submitMessage(GraspingJavaFXTopics.SelectedShape, shapeToCreate.ordinal());

         System.out.println("switch shape to " + shapeToCreate);
      }
   }

   private void createObject(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         if (shapeToCreate != null)
         {
            System.out.println("create " + shapeToCreate);
            switch (shapeToCreate)
            {
            case Sphere:
               Sphere3D sphere = new Sphere3D(controlPosition, sphereRadius.get().doubleValue());
               listOfShape3D.add(sphere);
               break;
            case Cylinder:
               Cylinder3D cylinder = new Cylinder3D(cylinderHeight.get().doubleValue(), cylinderRadius.get().doubleValue());
               cylinder.setPosition(controlPosition);
               cylinder.setOrientation(controlOrientation);
               listOfShape3D.add(cylinder);
               break;
            case Torus:
               Torus3D torus = new Torus3D(torusRadius.get().doubleValue(), torusTube.get().doubleValue());
               torus.setPosition(controlPosition);
               torus.setOrientation(controlOrientation);
               listOfShape3D.add(torus);
               break;
            case Box:
               Box3D box = new Box3D(controlPosition, controlOrientation, boxLength.get().doubleValue(), boxWidth.get().doubleValue(),
                                     boxHeight.get().doubleValue());
               listOfShape3D.add(box);
               break;
            default:
               break;
            }

            indexOfSelectedObject = listOfShape3D.size() - 1;
            shapeToCreate = null;
         }
         else
            System.out.println("shape should be selected.");
      }
   }

   private void switchSelectedObject(ButtonState state)
   {
      int numberOfObjects = listOfShape3D.size();

      if (numberOfObjects < 1)
         return;

      if (state == ButtonState.RELEASED)
         return;

      indexOfSelectedObject++;
      if (indexOfSelectedObject == numberOfObjects)
         indexOfSelectedObject = 0;

      listOfShape3D.get(indexOfSelectedObject).getPosition(controlPosition);
      listOfShape3D.get(indexOfSelectedObject).getOrientation(controlOrientation);
   }

   private void clearObjects(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         motionPreviewVisualizer.enable(false);
         listOfShape3D.clear();
         indexOfSelectedObject = 0;
      }
   }

   private void controlObject(Shape3D<?> shape3D)
   {
      shape3D.setPosition(controlPosition);
      shape3D.setOrientation(controlOrientation);

      if (shape3D instanceof Sphere3D)
      {
         ((Sphere3D) shape3D).setRadius(sphereRadius.get().doubleValue());
      }
      else if (shape3D instanceof Cylinder3D)
      {
         ((Cylinder3D) shape3D).setRadius(cylinderRadius.get().doubleValue());
         ((Cylinder3D) shape3D).setHeight(cylinderHeight.get().doubleValue());
      }
      else if (shape3D instanceof Torus3D)
      {
         ((Torus3D) shape3D).setRadii(torusRadius.get().doubleValue(), torusTube.get().doubleValue());
      }
      else if (shape3D instanceof Box3D)
      {
         ((Box3D) shape3D).setSize(boxLength.get().doubleValue(), boxWidth.get().doubleValue(), boxHeight.get().doubleValue());
         ;
      }
   }

   private Node createVisualizationObject(Shape3D<?> shape3D, boolean selected)
   {
      Point3D position = new Point3D(shape3D.getPosition());
      Quaternion orientation = new Quaternion(shape3D.getOrientation());

      MeshDataBuilder meshBuilder = new MeshDataBuilder();
      if (shape3D instanceof Sphere3D)
      {
         meshBuilder.addSphere(((Sphere3D) shape3D).getRadius(), new Point3D());
      }
      else if (shape3D instanceof Cylinder3D)
      {
         meshBuilder.addCylinder(((Cylinder3D) shape3D).getHeight(), ((Cylinder3D) shape3D).getRadius(), new Point3D());
      }
      else if (shape3D instanceof Box3D)
      {
         meshBuilder.addBox((float) ((Box3D) shape3D).getLength(), (float) ((Box3D) shape3D).getWidth(), (float) ((Box3D) shape3D).getHeight());
      }
      else if (shape3D instanceof Torus3D)
      {
         meshBuilder.addMesh(MeshDataGenerator.ArcTorus(0.0, 2 * Math.PI, ((Torus3D) shape3D).getRadius(), ((Torus3D) shape3D).getTubeRadius(), 10));
      }

      MeshDataHolder meshDataHolder = meshBuilder.generateMeshDataHolder();

      meshDataHolder = MeshDataHolder.rotate(meshDataHolder, orientation);
      meshDataHolder = MeshDataHolder.translate(meshDataHolder, position);

      Mesh mesh = JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder, true);

      MeshView meshView = new MeshView(mesh);
      if (selected)
         meshView.setMaterial(new PhongMaterial(Color.RED));
      else
         meshView.setMaterial(new PhongMaterial(Color.BEIGE));
      return meshView;
   }

   private void updateVisualizationObjects()
   {
      List<Node> objectsToPutReference = new ArrayList<Node>();
      for (int i = 0; i < listOfShape3D.size(); i++)
      {
         Shape3D<?> objectToVisualize = listOfShape3D.get(i);
         boolean selected = false;
         if (i == indexOfSelectedObject)
         {
            controlObject(objectToVisualize);
            selected = true;
         }

         Node node = createVisualizationObject(listOfShape3D.get(i), selected);
         objectsToPutReference.add(node);
      }

      JavaFXCoordinateSystem controlCoordinateSystem = new JavaFXCoordinateSystem(lengthOfControlFrame);
      Affine controlTransform = JavaFXTools.createAffineFromQuaternionAndTuple(new Quaternion(controlOrientation), controlPosition);
      controlCoordinateSystem.getTransforms().add(controlTransform);
      objectsToPutReference.add(controlCoordinateSystem);

      objectsToVisualizeReference.set(objectsToPutReference);

      List<Node> objectsToVisualize = objectsToVisualizeReference.getAndSet(null);
      ObservableList<Node> children = rootNode.getChildren();

      if (objectsToVisualize != null)
      {
         children.clear();
         children.addAll(objectsToVisualize);
      }
   }

   public Point3DReadOnly getControlObjectPosition()
   {
      return controlPosition;
   }

   public void start()
   {
      motionPreviewVisualizer.start();
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}