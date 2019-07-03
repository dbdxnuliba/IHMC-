package us.ihmc.atlas.behaviors.scsSensorSimulation;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.*;
import us.ihmc.mecano.multiBodySystem.*;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;

public class SensorOnlyRobot extends Robot
{
   private final LidarScanParameters lidarScanParameters;
   private final GimbalJoint gimbalJoint;
   private final FloatingJoint floatingJoint;

   public SensorOnlyRobot()
   {
      super("SensorOnlyRobot"); //initializes a robot

      System.out.println("Entering Sensor Only Robot");
      //creating the camera joint
      double height = 0.2;
      double radius = 0.05;

      floatingJoint = new FloatingJoint("rootJoint", new Vector3D(0.0, 0.0, 1.0), this);//, Axis.Z, Axis.X, Axis.Y);
      Link link1 = new Link("camera");
      link1.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
      Graphics3DObject linkGraphics1 = new Graphics3DObject(); //used to add rotation and translation
      linkGraphics1.addModelFile("models/hokuyo.dae",YoAppearance.Gold()); //camera joint
      link1.setLinkGraphics(linkGraphics1);
      floatingJoint.setLink(link1);
      floatingJoint.setPinned(true);
      Link link2 = new Link("lidar");
      link2.setMassAndRadiiOfGyration(1.0,radius,radius,radius);
      link2.setLinkGraphics(linkGraphics1);
      CameraMount robotCam = new CameraMount("camera", new Vector3D(radius + 0.001, 0.0, height / 2.0), this);  //offset length
      floatingJoint.addCameraMount(robotCam);


      gimbalJoint = new GimbalJoint("gimbalZ", "gimbalX", "gimbalY", new Vector3D(0.0, 0.0, 0.2), this, Axis.Z, Axis.X, Axis.Y);
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
      gimbalJoint.setLink(link2);
      gimbalJoint.setDamping(0.1);

      Graphics3DObject linkGraphics = new Graphics3DObject(); //used to add rotation and translation


      linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
      linkGraphics.translate(0, 0, -0.1);
      link2.setLinkGraphics(linkGraphics);
      this.addRootJoint(floatingJoint);

      floatingJoint.addJoint(gimbalJoint);
   }

   public FloatingJoint getCameraJoint()
   {
     return floatingJoint;

   }

   public GimbalJoint getLidarZJoint()
   {
      return gimbalJoint;
   }

   public PinJoint getLidarXJoint()
   {
      return (PinJoint) gimbalJoint.getChildrenJoints().get(0);  //.getChildrenJoints().get(0);
   }

   public LidarScanParameters getLidarScanParameters()
   {
      return lidarScanParameters;
   }
}
