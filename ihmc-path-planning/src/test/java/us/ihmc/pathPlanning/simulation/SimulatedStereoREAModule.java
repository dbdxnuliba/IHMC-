package us.ihmc.pathPlanning.simulation;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.ros2.Ros2Node;

public class SimulatedStereoREAModule
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "lidarScanPublisherNode");

   private static final double defaultUpdateFrequency = 1.5;
   private static final RGBDSensorDescription defaultCameraDescription = new MultiSenseParameters();

   private final SimulatedStereoCamera simulatedStereoCamera;
   private final LIDARBasedREAModule reaModule;

   public SimulatedStereoREAModule(ReferenceFrame cameraFrame) throws Exception
   {
      this(cameraFrame, defaultCameraDescription, defaultUpdateFrequency);
   }

   public SimulatedStereoREAModule(ReferenceFrame cameraFrame, RGBDSensorDescription sensorDescription, double updateFrequency) throws Exception
   {
      simulatedStereoCamera = new SimulatedStereoCamera(ros2Node, cameraFrame, sensorDescription, updateFrequency);
      reaModule = LIDARBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME);

      simulatedStereoCamera.start();
      reaModule.start();
   }

   public void destroy() throws Exception
   {
      ros2Node.destroy();
      simulatedStereoCamera.destroy();
      reaModule.stop();
   }
}
