package us.ihmc.atlas.behaviors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasBehaviorUIAndModule extends Application
{
   private BehaviorUI ui;  // created a Behavior UI object

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false); //create a robot model

      Messager behaviorMessager = RemoteBehaviorInterface.createForUI(NetworkParameters.getHost(NetworkParameterKeys.networkManager));  // creates a meassageer for the messages to be published by the behaviorUI module.
      //crates a kryo createclient on this side(Remote Behavior interface)and the kryo createserver is in behaviorModule

      ui = new BehaviorUI(primaryStage,
                          behaviorMessager,
                          drcRobotModel,
                          PubSubImplementation.FAST_RTPS); //instantiated it
      ui.show(); //method to stage a window
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      Platform.exit();
   }

   public static void main(String[] args)
   {
      ThreadTools.startAThread(() -> new AtlasBehaviorModule(), AtlasBehaviorUIAndModule.class.getSimpleName()); //launching UIModule and BehaviorModule simultaneously
      launch(args);
   }
}
