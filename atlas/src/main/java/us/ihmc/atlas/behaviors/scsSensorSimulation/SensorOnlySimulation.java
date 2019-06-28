package us.ihmc.atlas.behaviors.scsSensorSimulation;

import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.kinematicsSimulation.*;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SensorOnlySimulation
{

   private final SimulationConstructionSet scs;

   public SensorOnlySimulation()
   {
      SensorOnlyRobot robot = new SensorOnlyRobot();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100); //this gets set as well
      //AvatarKinematicsSimulation.createForManualTest(robotModel,false)

      SensorOnlyController controller = new SensorOnlyController(robot, yoGraphicsListRegistry, scs);
      robot.setController(controller); //no need to writing

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      CameraConfiguration camera = new CameraConfiguration("camera");
      camera.setCameraMount("camera");
      scs.setupCamera(camera);

      addSphere(2.0, 2.0, scs);
      addSphere(-2.0, 2.0, scs);
      addSphere(2.0, -2.0, scs);
      addSphere(-2.0, -2.0, scs);

      controller.initialize();

      scs.startOnAThread();
   }

  // public AvatarKinematicsSimulation setaks(DRCRobotModel robotModel)
   {
     // return AvatarKinematicsSimulation.createForManualTest(robotModel,false);
   }

   public SimulationConstructionSet getSCS()
   {
      return scs;
   }

   public void addSphere(double x, double y, SimulationConstructionSet scs)
   {
      Graphics3DObject sphere = new Graphics3DObject();
      sphere.translate(x, y, 0.5);
      sphere.addSphere(1.0, YoAppearance.Gray());
      scs.addStaticLinkGraphics(sphere);
   }

   public static void main(String[] args)
   {
      new SensorOnlySimulation();
   }
}
