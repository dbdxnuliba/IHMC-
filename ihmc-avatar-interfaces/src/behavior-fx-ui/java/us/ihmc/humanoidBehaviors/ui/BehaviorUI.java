package us.ihmc.humanoidBehaviors.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.ui.behaviors.DirectRobotUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.ExploreAreaBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.FancyPosesBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.PatrolBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.PlannerParametersUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.StepInPlaceBehaviorUIController;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.humanoidBehaviors.ui.tools.LocalParameterServer;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private static boolean showLivePlanarRegionsGraphic = true;

   private final Stage primaryStage;
   private final BorderPane mainPane;

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   @FXML private PatrolBehaviorUIController patrolBehaviorUIController;
   @FXML private StepInPlaceBehaviorUIController stepInPlaceBehaviorUIController;
   @FXML private FancyPosesBehaviorUIController fancyPosesBehaviorUIController;
   @FXML private ExploreAreaBehaviorUIController exploreAreaBehaviorUIController;
   @FXML private PlannerParametersUIController plannerParametersUIController;
   @FXML private DirectRobotUIController directRobotUIController;

   public BehaviorUI(Stage primaryStage,
                     Messager behaviorMessager,
                     DRCRobotModel robotModel,
                     PubSubImplementation pubSubImplementation) throws Exception
   {
      this.primaryStage = primaryStage;

      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_ui");

      if (LabelGraphic.TUNING_MODE)
      {
         LocalParameterServer parameterServer = new LocalParameterServer(getClass(), 16784);
         LabelGraphic.initializeYoVariables(parameterServer.getRegistry());
         parameterServer.start();
      }

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(0.05, 2000.0,true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();
      SubScene subScene = view3dFactory.getSubScene();
      Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

      stepInPlaceBehaviorUIController.init(behaviorMessager);
      fancyPosesBehaviorUIController.init(behaviorMessager);
      exploreAreaBehaviorUIController.init(subScene, behaviorMessager, robotModel);
      patrolBehaviorUIController.init(subScene, behaviorMessager, robotModel);
      plannerParametersUIController.init(behaviorMessager, robotModel);
      directRobotUIController.init(ros2Node, robotModel);

      view3dFactory.addNodeToView(patrolBehaviorUIController);
      view3dFactory.addNodeToView(exploreAreaBehaviorUIController);
      
      if (showLivePlanarRegionsGraphic)
      {
         view3dFactory.addNodeToView(new LivePlanarRegionsGraphic(ros2Node));
      }

      view3dFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));

      mainPane.setCenter(subSceneWrappedInsidePane);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1350, 900);

      primaryStage.setScene(mainScene);
   }

   public void show()
   {
      primaryStage.show();
   }

   public static void claimEditing(Object claimingEditor)
   {
      if (BehaviorUI.ACTIVE_EDITOR != null)
      {
         throw new RuntimeException("Only one editor may be active at a time.");
      }
      else
      {
         BehaviorUI.ACTIVE_EDITOR = claimingEditor;
         LogTools.debug("editor activated: {}", claimingEditor.getClass().getSimpleName());
      }
   }
}
