package us.ihmc.avatar.kinematicsSimulation;

import boofcv.struct.calib.*;
import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.environments.*;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.codecs.generated.*;
import us.ihmc.codecs.generated.YUVPicture.*;
import us.ihmc.codecs.yuv.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.*;
import us.ihmc.communication.producers.*;
import us.ihmc.euclid.*;
import us.ihmc.euclid.transform.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.euclid.tuple4D.interfaces.*;
import us.ihmc.graphicsDescription.*;
import us.ihmc.graphicsDescription.appearance.*;
import us.ihmc.graphicsDescription.image.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.jMonkeyEngineToolkit.*;
import us.ihmc.jMonkeyEngineToolkit.camera.*;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.lidar.*;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.*;
import us.ihmc.simulationconstructionset.gui.actions.*;
import us.ihmc.simulationconstructionset.gui.camera.*;
import us.ihmc.simulationconstructionset.simulatedSensors.*;
import us.ihmc.tools.*;
import us.ihmc.tools.functional.FunctionalTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.awt.image.*;
import java.io.*;
import java.nio.*;
import java.util.Arrays;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class AvatarKinematicsSimulation
{
   private static final double DT = UnitConversions.hertzToSeconds(70);
   public static final double PLAYBACK_SPEED = 10.0;
   private final DRCRobotModel robotModel;
   private final AvatarKinematicsSimulationController avatarKinematicsSimulationController;
   private final ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                   DefaultExceptionHandler.PRINT_MESSAGE,
                                                                                                   5);
   private final Ros2Node ros2Node;
   private final IHMCROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final IHMCROS2Publisher<WalkingStatusMessage> walkingStatusPublisher;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private double yoVariableServerTime = 0.0;
   private YoVariableServer yoVariableServer;
   private ScheduledFuture<?> yoVariableServerScheduled;
   private StandardSimulationGUI myGUI;
   private CameraConfigurationList cameraConfigurationList = new CameraConfigurationList();
   private AvatarKinematicsSimulation aks;

   private final int width = 1024;
   private final int height = 544;
   private final int framesPerSecond = 25;


   private IHMCROS2Publisher<VideoPacket> scsCameraPublisher;

   public static void createForManualTest(DRCRobotModel robotModel, boolean createYoVariableServer)
   {
      //create(robotModel, createYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public static void createForManualTestwscs(DRCRobotModel robotModel, boolean createYoVariableServer, SimulationConstructionSet scs)
   {
      create(robotModel, createYoVariableServer, PubSubImplementation.FAST_RTPS, scs);
   }

   public static void createForAutomatedTest(DRCRobotModel robotModel, boolean createYoVariableServer)
   {
      //create(robotModel, createYoVariableServer, PubSubImplementation.INTRAPROCESS);
   }

   private static void create(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation, SimulationConstructionSet scs)
   {
      boolean DEBUG = false;
      if(!DEBUG)
      {
         new AvatarKinematicsSimulation(robotModel, createYoVariableServer, pubSubImplementation);
      }
      else
      {
         //new AvatarKinematicsSimulation(robotModel, createYoVariableServer, pubSubImplementation,scs);
      }
   }




   public AvatarKinematicsSimulation(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this.robotModel = robotModel;
      robotModel.getSimulateDT();

      // instantiate some existing controller ROS2 API?
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, HighLevelHumanoidControllerFactory.ROS2_ID.getNodeName("kinematic"));
      scsCameraPublisher = new IHMCROS2Publisher<>(ros2Node, VideoPacket.class);

      robotConfigurationDataPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                RobotConfigurationData.class,
                                                                robotModel.getSimpleRobotName(),
                                                                HighLevelHumanoidControllerFactory.ROS2_ID);
      walkingStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                       WalkingStatusMessage.class,
                                                       robotModel.getSimpleRobotName(),
                                                       HighLevelHumanoidControllerFactory.ROS2_ID);

      new ROS2Callback<>(ros2Node,
                         FootstepDataListMessage.class,
                         robotModel.getSimpleRobotName(),
                         HighLevelHumanoidControllerFactory.ROS2_ID,
                         this::acceptFootstepDataListMessage);


      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      avatarKinematicsSimulationController = new AvatarKinematicsSimulationController(robotModel,
                                                                                      robotInitialSetup.getInitialPelvisPose(),
                                                                                      robotInitialSetup.getInitialJointAngles(),
                                                                                      DT,
                                                                                      yoGraphicsListRegistry,
                                                                                      registry);

      FunctionalTools.runIfTrue(createYoVariableServer, this::createYoVariableServer);

      avatarKinematicsSimulationController.initialize();

      //SensorOnlySimulation sensorOnlySimulation = new SensorOnlySimulation();
      //SimulationConstructionSet scs = sensorOnlySimulation.getSCS();
      sensorinfo(robotModel,ros2Node, width, height);

      scheduler.schedule(this::controllerTick, Conversions.secondsToNanoseconds(DT / PLAYBACK_SPEED), TimeUnit.NANOSECONDS);
   }

   private void controllerTick()
   {
      avatarKinematicsSimulationController.doControl();

      robotConfigurationDataPublisher.publish(extractRobotConfigurationData(avatarKinematicsSimulationController.getFullRobotModel()));

      if (avatarKinematicsSimulationController.getWalkingCompletedNotification().poll())
      {
         walkingStatusPublisher.publish(WalkingStatus.COMPLETED.createMessage());
      }
   }

   private void acceptFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      avatarKinematicsSimulationController.getInputManager().submitMessage(footstepDataListMessage);
   }

   private void createYoVariableServer()
   {
      yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), new DataServerSettings(false), 0.01);
      yoVariableServer.setMainRegistry(registry, robotModel.createFullRobotModel().getElevator(), yoGraphicsListRegistry);
      ThreadTools.startAThread(() -> yoVariableServer.start(), getClass().getSimpleName() + "YoVariableServer");

      yoVariableServerScheduled = scheduler.schedule(this::yoVariableUpdateThread, 1, TimeUnit.MILLISECONDS);
   }

   private void yoVariableUpdateThread()
   {
      if (!Thread.interrupted())
      {
         yoVariableServerTime += Conversions.millisecondsToSeconds(1);
         yoVariableServer.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
      }
   }

   public static RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints,
                                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                                           fullRobotModel.getIMUDefinitions());
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootTranslation().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());

      return robotConfigurationData;
   }

   //write native code that takes in camera info similar to the one done in scs

   //public void setDt(double simulateDT, int recordFrequency)
   //{

   //}
   public void sensorinfo(DRCRobotModel robotModel, Ros2Node ros2Node, int width, int height)
   {
      //RobotSesnor sesnoronlyRobot = new RobotSesnor();
      //YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      //Simulation mysimulation = new Simulation();


      ROS2Input<RobotConfigurationData> robotConfigurationData = new ROS2Input<>(ros2Node,
                                                                                RobotConfigurationData.class,
                                                                                robotModel.getSimpleRobotName(),
                                                                                HighLevelHumanoidControllerFactory.ROS2_ID);

      //AvatarKinematicsSimulation.createForManualTest(robotModel,false);

      CameraConfiguration camera = new CameraConfiguration("camera");
      camera.setCameraMount("camera");

      cameraConfigurationList.addCameraConfiguration(camera);


      //scs.startStreamingVideoData(camera, width, height, new VideoDataServerImageCallback(new VideoPacketCallback()),
      //                            () -> robotConfigurationData.getLatest().getSyncTimestamp(),
      //                            framesPerSecond);

      AvatarKinematicsSimulation.startstreamingvideo(camera, width, height, new VideoDataServerImageCallback(new VideoPacketCallback()),
                              () -> robotConfigurationData.getLatest().getSyncTimestamp(),
                              framesPerSecond);

      //getRobotModel().getSimulateDT();


   }

   //write a method similar to sartstreamingvideodata

   public static void startstreamingvideo(CameraConfiguration cameraConfiguration, int width, int height, ImageCallback imageCallback,
                                   TimestampProvider timestampProvider, int framesPerSecond)
   {
      CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder = new CameraTrackAndDollyYoVariablesHolder(null);
      new OffscreenBufferVideoServer(SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true), new CameraMountList(), cameraConfiguration, cameraTrackingAndDollyPositionHolder, width, height,
                                     imageCallback, timestampProvider, framesPerSecond);
   }

   public void setupCamera(CameraConfiguration cameraConfiguration)
   {



      //supportedGraphics3DAdapter.instatiateDefaultGraphicsAdapter(true)

      /*EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            createGUI(SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true));
         }
      });

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            standardGUIActions.setupCameraMenu(cameraConfigurationList, getStandardSimulationGUI());
         }
      });*/
   }

   /*private void createGUI(Graphics3DAdapter graphics3DAdapter)
   {
      myGUI = new StandardSimulationGUI(graphics3DAdapter, simulationSynchronizer, standardAllCommandsExecutor, null, this, this, robots, myDataBuffer,
                                        varGroupList, jApplet, rootRegistry);
   }*/
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   /*
   private class RobotSesnor extends Robot
   {
      private final LidarScanParameters lidarScanParameters;
      private final GimbalJoint gimbalJoint;

      public RobotSesnor()
      {
         super("SensorOnlyRobot");

         double height = 0.2;
         double radius = 0.05;

         gimbalJoint = new GimbalJoint("gimbalZ", "gimbalX", "gimbalY", new Vector3D(0.0, 0.0, 1.0), this, Axis.Z, Axis.X, Axis.Y);
         Link link = new Link("lidar");
         link.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
         Graphics3DObject linkGraphics = new Graphics3DObject();
         link.setLinkGraphics(linkGraphics);
         gimbalJoint.setLink(link);
         gimbalJoint.setDamping(1.0);

         CameraMount robotCam = new CameraMount("camera", new Vector3D(radius + 0.001, 0.0, height / 2.0), this);
         gimbalJoint.addCameraMount(robotCam);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(new Vector3D(radius + 0.001, 0.0, height / 2.0));
         lidarScanParameters = new LidarScanParameters(720, (float) (-Math.PI / 2), (float) (Math.PI / 2), 0f, 0.1f, 30.0f, 0f);
         LidarSensorDescription lidarSensorDescription = new LidarSensorDescription("lidar", transform);
         lidarSensorDescription.setPointsPerSweep(lidarScanParameters.getPointsPerSweep());
         lidarSensorDescription.setScanHeight(lidarScanParameters.getScanHeight());
         lidarSensorDescription.setSweepYawLimits(lidarScanParameters.getSweepYawMin(), lidarScanParameters.getSweepYawMax());
         lidarSensorDescription.setHeightPitchLimits(lidarScanParameters.heightPitchMin, lidarScanParameters.heightPitchMax);
         lidarSensorDescription.setRangeLimits(lidarScanParameters.getMinRange(), lidarScanParameters.getMaxRange());
         LidarMount lidarMount = new LidarMount(lidarSensorDescription);

         gimbalJoint.addLidarMount(lidarMount);

         linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
         linkGraphics.translate(0, 0, -0.1);
         link.setLinkGraphics(linkGraphics);
         this.addRootJoint(gimbalJoint);
      }

      public GimbalJoint getLidarZJoint()
      {
         return gimbalJoint;
      }

      public PinJoint getLidarXJoint()
      {
         return (PinJoint) gimbalJoint.getChildrenJoints().get(0);
      }

      public LidarScanParameters getLidarScanParameters()
      {
         return lidarScanParameters;
      }
   }*/
   private static final Object hackyLockBecauseJPEGEncoderIsNotThreadsafe = new Object();

   class VideoPacketCallback implements VideoDataServer
   {
      private final YUVPictureConverter converter = new YUVPictureConverter();
      private final JPEGEncoder encoder = new JPEGEncoder();

      @Override
      public void onFrame(VideoSource videoSource,
                          BufferedImage bufferedImage,
                          long timeStamp,
                          Point3DReadOnly cameraPosition,
                          QuaternionReadOnly cameraOrientation,
                          IntrinsicParameters intrinsicParameters)
      {

         YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);
         try
         {
            ByteBuffer buffer;
            synchronized (hackyLockBecauseJPEGEncoderIsNotThreadsafe)
            {
               buffer = encoder.encode(picture, 75);
            }
            byte[] data = new byte[buffer.remaining()];
            buffer.get(data);
            VideoPacket videoPacket = HumanoidMessageTools.createVideoPacket(videoSource,
                                                                             timeStamp,
                                                                             data,
                                                                             cameraPosition,
                                                                             cameraOrientation,
                                                                             intrinsicParameters);
            scsCameraPublisher.publish(videoPacket);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         picture.delete();
      }

      @Override
      public boolean isConnected()
      {
         return true; // do nothing
      }
   }

}



