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
      linkGraphics1.addModelFile("models/hokuyo.dae",YoAppearance.Gold());
      link1.setLinkGraphics(linkGraphics1);
      floatingJoint.setLink(link1);
      floatingJoint.setPinned(true);
      //floatingJoint.setDamping(1.0);
      Link link2 = new Link("lidar");
      link2.setMassAndRadiiOfGyration(1.0,radius,radius,radius);
      link2.setLinkGraphics(linkGraphics1);
      CameraMount robotCam = new CameraMount("camera", new Vector3D(radius + 0.001, 0.0, height / 2.0), this);  //offset length
      floatingJoint.addCameraMount(robotCam);


      //create a floating joint

      /*floatingJoint = new FloatingJoint("root", new Vector3D(0.0,0.0,1.0),this);//, Axis.Z,Axis.X, Axis.Y);
      Link rootlink = new Link("lidar");
      rootlink.setMassAndRadiiOfGyration(1.0, radius,radius, radius);
      Graphics3DObject linkgraphics = new Graphics3DObject();
      rootlink.setLinkGraphics(linkgraphics);
      floatingJoint.setLink(rootlink);



      /*gimbalJoint = new GimbalJoint("gimbalZ", "gimbalX", "gimbalY", new Vector3D(0.0, 0.0, 1.0), this, Axis.Z, Axis.X, Axis.Y);
      //Link link = new Link("lidar");
      //link.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
      //Graphics3DObject linkGraphics = new Graphics3DObject();
      //link.setLinkGraphics(linkGraphics);
      //gimbalJoint.setLink(link);
      gimbalJoint.setDamping(1.0);

      CameraMount robotCam = new CameraMount("camera", new Vector3D(radius + 0.001, 0.0, height / 2.0), this);  //offset length
      //gimbalJoint.addCameraMount(robotCam);
      floatingJoint.addCameraMount(robotCam);
      //this.addRootJoint(floatingJoint);

      Vector3D jointOffset1 = new Vector3D(0.12, 1.17, 3.125);
      double mass1 = 1.12;
      Vector3D comOffset1 = new Vector3D(0.1, 1.11, 3.79);

      Matrix3D momentOfInertia1 = new Matrix3D();
      momentOfInertia1.setM00(1.95);
      momentOfInertia1.setM11(3.93);
      momentOfInertia1.setM22(7.91);

      Vector3D jointOffset2 = new Vector3D(-0.4, 1.76, 1.1);
      double mass2 = 1.12;
      Vector3D comOffset2 = new Vector3D();
      Matrix3D momentOfInertia2 = new Matrix3D();
      momentOfInertia1.setM00(51.95);
      momentOfInertia1.setM11(0.93);
      momentOfInertia1.setM22(7.51);

      PinJoint joint1 = new PinJoint("joint1", jointOffset1, this, Axis.X);
      Link link1 = new Link("link1");
      link1.setMass(mass1);
      link1.setMomentOfInertia(momentOfInertia1);
      link1.setComOffset(comOffset1);
      joint1.setLink(link1);

      PinJoint joint2 = new PinJoint("joint2", jointOffset2, this, Axis.X);
      Link link2 = new Link("link2");
      link2.setMass(mass2);
      link2.setMomentOfInertia(momentOfInertia2);
      link2.setComOffset(comOffset2);
      joint2.setLink(link2);


      floatingJoint.addJoint(joint1);
      floatingJoint.addJoint(joint2);

      linkgraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
      linkgraphics.translate(0, 0, -0.1);
      rootlink.setLinkGraphics(linkgraphics);
      this.addRootJoint(gimbalJoint);*/

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

      //linkGraphics.addCoordinateSystem(0.0);
      linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
      linkGraphics.translate(0, 0, -0.1);
      link2.setLinkGraphics(linkGraphics);
      this.addRootJoint(floatingJoint);
      //this.setGravity(0.0);
//      this.addRootJoint(gimbalJoint);
      floatingJoint.addJoint(gimbalJoint);
   }

   public FloatingJoint getCameraJoint()
   {
     return floatingJoint;
     //return (Joint) floatingJoint.getChildrenJoints().get(0);  //.getChildrenJoints().get(0);

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
