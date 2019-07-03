package us.ihmc.atlas.behaviors.scsSensorSimulation;

import us.ihmc.atlas.*;
import us.ihmc.atlas.behaviors.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.communication.*;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.pubsub.DomainFactory.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationconstructionset.*;

public class SensorOnlySimulation
{

   private final SimulationConstructionSet scs;

   public SensorOnlySimulation(RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames)
   {
      System.out.println("Entering Sensor Only Simulation");
      SensorOnlyRobot robot = new SensorOnlyRobot();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);

      SensorOnlyController controller = new SensorOnlyController(robot, yoGraphicsListRegistry, scs, remoteSyncedHumanoidFrames);
      robot.setController(controller);


      //camer and lidar attached to the robot in sensorOnlyRobot class
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      CameraConfiguration camera = new CameraConfiguration("camera");
      camera.setCameraMount("camera");
      scs.setupCamera(camera);


      addSphere(6.0, 6.0, scs);
      addSphere(-6.0, 6.0, scs);
      addSphere(6.0, -6.0, scs);
      addSphere(-6.0, -6.0, scs);
      System.out.println("Initializing controller ");
      controller.initialize();

      scs.startOnAThread();

      scs.simulate();
   }

   public SimulationConstructionSet getSCS()
   {
      return scs;
   }

   public void addSphere(double x, double y, SimulationConstructionSet scs)
   {
      Graphics3DObject sphere = new Graphics3DObject();
      sphere.translate(x, y, 0.5);
      sphere.addSphere(3.0, YoAppearance.Gray());
      scs.addStaticLinkGraphics(sphere);
   }

 /*public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasBehaviorModule.ATLAS_VERSION, RobotTarget.SCS,false);
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "abc");
      RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);
      new SensorOnlySimulation(remoteSyncedHumanoidFrames);
   }*/
}
