package us.ihmc.atlas.behaviors;

import boofcv.struct.calib.*;
import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasKinematicSimWithCamera.*;
import us.ihmc.atlas.behaviors.scsSensorSimulation.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.codecs.generated.*;
import us.ihmc.codecs.generated.YUVPicture.*;
import us.ihmc.codecs.yuv.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.*;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.producers.*;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.euclid.tuple4D.interfaces.*;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.jMonkeyEngineToolkit.camera.*;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.*;
import us.ihmc.simulationconstructionset.*;

import java.awt.image.*;
import java.io.*;
import java.nio.*;

public class AtlasBehaviorUIAndModule extends Application
{

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);

      Messager behaviorMessager = RemoteBehaviorInterface.createForUI(NetworkParameters.getHost(NetworkParameterKeys.networkManager));

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "behavior_camera");
      //scsCameraPublisher = new IHMCROS2Publisher<>(ros2Node, VideoPacket.class);



      ui = new BehaviorUI(primaryStage,
                          behaviorMessager,
                          drcRobotModel,
                          PubSubImplementation.FAST_RTPS);
      ui.show();
   }


   @Override
   public void stop() throws Exception
   {
      super.stop();
      Platform.exit();
   }

   public static void main(String[] args)
   {
      ThreadTools.startAThread(() -> new AtlasBehaviorModule(), AtlasBehaviorUIAndModule.class.getSimpleName());
      launch(args);
   }
}
