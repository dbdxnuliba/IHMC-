package us.ihmc.atlas.behaviors.scsSensorSimulation;

import us.ihmc.atlas.*;
import us.ihmc.atlas.behaviors.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.kinematicsSimulation.*;
import us.ihmc.graphicsDescription.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.pubsub.DomainFactory.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationconstructionset.*;

import java.util.*;

public class SensorOnlySimulation
{

   private final SimulationConstructionSet scs;
   private final AvatarKinematicsSimulation aks = new AvatarKinematicsSimulation(new AtlasRobotModel(AtlasBehaviorModule.ATLAS_VERSION, RobotTarget.SCS,false)
         , false, PubSubImplementation.FAST_RTPS);
   Simulation mysimulation;

   private SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();

   private ArrayList<PlaybackListener> playbackListeners = null;

   public SensorOnlySimulation()
   {
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SensorOnlyRobot robot = new SensorOnlyRobot();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();


      scs = new SimulationConstructionSet(robot);
      mysimulation = new Simulation(robot, scs.getSimulationConstructionSetParameters().getDataBufferSize());
      //scs.setDT(0.0001, 100); //this gets set as well
      aks.setDt(0.0001,100);
      //AvatarKinematicsSimulation.createForManualTest(robotModel,false)

      SensorOnlyController controller = new SensorOnlyController(robot, yoGraphicsListRegistry);//, scs);
      robot.setController(controller); //no need to writing

      //aks.addYoGraphicsListRegistry(yoGraphicsListRegistry);

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

   public void setDT(double simulateDT, int recordFrequency)
   {
      mysimulation.setDT(simulateDT, recordFrequency);
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry, boolean updateFromSimulationThread)
   {
      if (yoGraphicsListRegistry.areYoGraphicsRegistered())
         throw new RuntimeException("Already added this YoGraphicsListRegistry To SimulationConstructionSet: " + yoGraphicsListRegistry);

      ArrayList<GraphicsUpdatable> graphicsUpdatablesToUpdateInAPlaybackListener = yoGraphicsListRegistry.getGraphicsUpdatablesToUpdateInAPlaybackListener();
      if (graphicsUpdatablesToUpdateInAPlaybackListener != null)
      {

         GraphicsUpdatablePlaybackListener playbackListener = new GraphicsUpdatablePlaybackListener(graphicsUpdatablesToUpdateInAPlaybackListener);
         this.attachPlaybackListener(playbackListener);
      }

//      if (myGUI != null)
//         myGUI.addYoGraphicsListRegistry(yoGraphicsListRegistry, updateFromSimulationThread);

      List<YoGraphicsList> yoGraphicsLists = yoGraphicsListRegistry.getYoGraphicsLists();

      if (yoGraphicsLists == null)
         return;

//      yoGraphicListRegistries.add(yoGraphicsListRegistry);
   }

   public void attachPlaybackListener(PlaybackListener playbackListener)
   {
      if (playbackListeners == null)
      {
         this.playbackListeners = new ArrayList<PlaybackListener>();
      }

      playbackListeners.add(playbackListener);

//      myDataBuffer.attachIndexChangedListener(playbackListener);
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
