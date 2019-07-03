package us.ihmc.atlas.behaviors.scsSensorSimulation;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.*;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.Random;
import java.util.concurrent.LinkedBlockingQueue;

public class SensorOnlyController implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   //private FloatingJoint tmpJoint;
   private final SensorOnlyRobot robot;
   private final SimulationConstructionSet scs;
   private final LidarScanParameters lidarScanParameters;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private GPULidar gpuLidar;
   private LinkedBlockingQueue<LidarScan> gpuLidarScanBuffer = new LinkedBlockingQueue<>();
   private YoDouble tauLidarZ;
   private YoDouble tauLidarX;
   private YoDouble qLidarZ;
   private YoDouble qLidarY;
   private YoDouble qLidarX;

   private final YoFramePoint3D point = new YoFramePoint3D("point", ReferenceFrame.getWorldFrame(), registry);
   private final BagOfBalls bagOfBalls;

   private PDController pdControllerZ;
   private PDController pdControllerX;
   private final YoDouble proportionalGain;
   private final YoDouble derivativeGain;
   private double lastPositionZ;
   private double lastPositionY;
   private double lastPositionX;
   private double lastTime;
   private double desiredZRate = 0.3;
   private final static double DESIRED_X_RATE = 0.3;

   public SensorOnlyController(SensorOnlyRobot robot,
                               YoGraphicsListRegistry yoGraphicsListRegistry,
                               SimulationConstructionSet scs,
                               RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames)
   {
      this.robot = robot; //creates the controller for the rest of the robot
      this.scs = scs;
      this.lidarScanParameters = robot.getLidarScanParameters();
      this.remoteSyncedHumanoidFrames = remoteSyncedHumanoidFrames; //getting instantiated

      YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("point", point, 0.01, YoAppearance.Purple());
      yoGraphicsListRegistry.registerYoGraphic("test", yoGraphicPosition);

      bagOfBalls = new BagOfBalls(lidarScanParameters.getPointsPerSweep(), 0.005, YoAppearance.AliceBlue(), registry, yoGraphicsListRegistry);

      //tmpJoint = new FloatingJoint("camera1",new Vector3D(), robot);
      tauLidarZ = (YoDouble) robot.getVariable("tau_gimbalZ");
      tauLidarX = (YoDouble) robot.getVariable("tau_gimbalX");

      qLidarZ = (YoDouble) robot.getVariable("q_gimbalZ");
      qLidarY = (YoDouble) robot.getVariable("q_gimbalY");
      qLidarX = (YoDouble) robot.getVariable("q_gimbalX");

      proportionalGain = new YoDouble("lidarPGain", registry);
      derivativeGain = new YoDouble("lidarDGain", registry);

      pdControllerX = new PDController(proportionalGain, derivativeGain, "LidarControllerX", registry);
      pdControllerZ = new PDController(proportionalGain, derivativeGain, "LidarControllerZ", registry);
      pdControllerZ.setProportionalGain(1.0);
      pdControllerZ.setDerivativeGain(1.0);

      lastPositionZ = qLidarZ.getDoubleValue();
      lastPositionY = qLidarY.getDoubleValue();
      lastPositionX = qLidarX.getDoubleValue();
      lastTime = scs.getTime();
      System.out.println("Entering Sensor Only Controller");
   }

   public void initialize()
   {
      startGPULidar();
   }

   private void startGPULidar()
   {
      gpuLidar = scs.getGraphics3dAdapter().createGPULidar(lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getScanHeight(),
                                                           lidarScanParameters.getFieldOfView(), lidarScanParameters.getMinRange(),
                                                           lidarScanParameters.getMaxRange());
      gpuLidar.addGPULidarListener((scan, currentTransform,
                                    time) -> gpuLidarScanBuffer.add(new LidarScan(lidarScanParameters, new RigidBodyTransform(currentTransform),
                                                                                  new RigidBodyTransform(currentTransform), scan)));
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   private final Random random = new Random(1776L);

   public void doControl()
   {

      FramePose3DReadOnly neckPose = remoteSyncedHumanoidFrames.quickPollPoseReadOnly(frames -> frames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH)); //getting the neckpose from the Atlas in kinematicSimWithCamera


     // FramePose3DReadOnly neckPose = remoteSyncedHumanoidFrames.quickPollPoseReadOnly(frames -> frames.getLegJointFrame(RobotSide.LEFT, LegJointName.HIP_YAW));
      /*double currentTime = scs.getTime();
      double dt = currentTime - lastTime;
      lastTime = currentTime;

      double currentZPosition = qLidarZ.getValueAsDouble();
      double desiredZPosition = lastPositionZ + desiredZRate * dt;
      double currentZRate = (currentZPosition - lastPositionZ) * dt;

      double desiredXRate = Math.cos(currentZPosition) * DESIRED_X_RATE;
      double currentXPosition = qLidarX.getValueAsDouble();
      double desiredXPosition = lastPositionX + desiredXRate * dt;
      double currentXRate = (currentXPosition - lastPositionX) * dt;

      double zCorrectionSum = pdControllerZ.compute(currentZPosition, desiredZPosition, currentZRate, desiredZRate);
      double xCorrectionSum = pdControllerX.compute(currentXPosition, desiredXPosition, currentXRate, desiredXRate);

      /*if(neckPose.getX() != -0.10066621529019597)
      {
         int t = 0;
         if(t%50 == 0)
         {
            System.out.println("-------------------------------------------------------------------------- \n");
            //System.out.println("Entering Sensor Only Controller - do control loop ; I should be printed infinite times");
            System.out.println("Printing neckPose Parameters : " + neckPose.getX() + " " + neckPose.getY() + " " + neckPose.getZ());
            t++;
         }
      }*/

      //tmpJoint.setPosition(neckPose.getX(),neckPose.getY(),neckPose.getZ());
      robot.getCameraJoint().setPosition(neckPose.getX(),neckPose.getY(),neckPose.getZ());
      robot.getCameraJoint().setYawPitchRoll(neckPose.getYaw(), neckPose.getPitch(), neckPose.getRoll());
      //tauLidarZ.set(zCorrectionSum);
      //tauLidarX.set(xCorrectionSum);

      lastPositionX = neckPose.getX();
      lastPositionY = neckPose.getY();
      lastPositionZ = neckPose.getZ();



      RigidBodyTransform transform = new RigidBodyTransform();


      robot.getLidarXJoint().getTransformToWorld(transform); //transforms from transform to world frame
      //robot.getCameraJoint().getTransformToWorld(transform);

      gpuLidar.setTransformFromWorld(transform, 0);

      //robot.getLidarZJoint().setQ(neckPose.getZ());
      //robot.get
      //RigidBodyTransform transform = new RigidBodyTransform();
      //neckPose.get(transform);


//      robot.getLidarXJoint().getTransformToWorld(transform);

//      gpuLidar.setTransformFromWorld(transform, 0);

      while (!gpuLidarScanBuffer.isEmpty())
      {
         LidarScan scan = gpuLidarScanBuffer.poll();

         for (int i = 0; i < scan.size(); i++)
         {
            bagOfBalls.setBallLoop(new FramePoint3D(ReferenceFrame.getWorldFrame(), scan.getPoint(i)), YoAppearance.randomColor(random));
         }
      }
   }
}
