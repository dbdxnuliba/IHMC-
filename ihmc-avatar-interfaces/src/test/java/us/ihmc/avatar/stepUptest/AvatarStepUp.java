package us.ihmc.avatar.stepUptest;

import static us.ihmc.robotics.Assert.*;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.*;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.TrajectoryStatusMessageHelper.*;
import us.ihmc.commons.thread.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.euclid.tuple4D.interfaces.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.idl.IDLSequence.Double;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.*;
import us.ihmc.robotics.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.trajectories.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.*;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationToolkit.controllers.*;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;
import us.ihmc.simulationconstructionset.util.simulationTesting.*;
import us.ihmc.simulationconstructionset.Joint;

import us.ihmc.tools.*;

import java.util.*;

public abstract class AvatarStepUp implements MultiRobotTestInterface
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   //private final AtlasR

   @BeforeEach
   public  void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach

   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if(drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + "after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   //start writing your step up code here now
   @Test
   public void stepUpSmall() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.3;
      double swingHeight = 0.15; //maybe needs to be changed
      walkingStair(stepHeight, swingHeight);

   }

   @Test
   public void stepUpBig() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.5;
      //OffsetAndYawRobotInitialSetup
      //drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.5, 0.0, 0.0, 0.0));
      //walkingStair(stepHeight);
   }

   @Test
   private void walkingStair(double stepHeight, double swingHeight) throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = setTestEnvironment(stepHeight);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      FootstepDataListMessage footsteps = createFootSteps(robotModel, stepHeight, swingHeight);
      PelvisHeightTrajectoryMessage pelvisDHeight = createPelvisZUp(stepHeight); //hits the stairs so modify the waypoints for the foot so that you can take step a bit back and then go ahead
      ChestTrajectoryMessage chestTrajectoryPoints = createChestTrajectory(ReferenceFrame.getWorldFrame(),drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame()); //create desired chest trajectory
      ArmTrajectoryMessage[] armTrajectoryMessages = createArmTrajectory();

      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.publishToController(pelvisDHeight);
      drcSimulationTestHelper.publishToController(chestTrajectoryPoints);

      int i = 0; //variable for iterating over eac arm orientation message (an array of double)
      do
      {

         drcSimulationTestHelper.publishToController(armTrajectoryMessages[i]);
         ++i;
      }
      while (i<2);

      //Object<Point3D> waypoints = footsteps.getFootstepDataList().get(1).getCustomPositionWaypoints();
      //Double Proportion = footsteps.getFootstepDataList().get(1).getCustomWayPointProportions();
      //System.out.println(waypoints);
      //System.out.println(Proportion);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTouchdownTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      // robot fell
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime +2.0*initialFinalTransfer + 11.0));

      // robot did not fall but did not reach goal
      assertreached(footsteps);

      ThreadTools.sleepForever(); //does not kill the simulation
   }

   private ArmTrajectoryMessage[] createArmTrajectory()
   {
      ArmTrajectoryMessage[] armPoses = new ArmTrajectoryMessage[]{new ArmTrajectoryMessage(), new ArmTrajectoryMessage()};
      double trajectoryTime = 0.5;
      double[][] armJoints = new double[][] {{-2.5, -0.8, 1.12, 1.48, 0.9, -1.0, -2.8},{0.44, 0.02, 2.7, -1.78, 0.3, -1.5, -0.5}};
      RobotSide[] side = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 2);
      // this message commands the controller to move an arm in JOINTSPACE to the desired joint angles while going through the specified trajectory points
      for(int j=0;j < 2 ; ++j)
      {
         double[] desiredArmPose = armJoints[j];
         ArmTrajectoryMessage armPose = HumanoidMessageTools.createArmTrajectoryMessage(side[j], trajectoryTime, desiredArmPose);
         armPoses[j] = armPose;
      }
      return armPoses;
   }

   private ChestTrajectoryMessage createChestTrajectory(ReferenceFrame dataframe, ReferenceFrame trajectoryFrame)
   {
      double trajectoryTime = 0.5;
      FrameQuaternion chestOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      double leanAngle = 2.0; //original values 20.0 and yaw was -2.36
      chestOrientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0,Math.toRadians(leanAngle), 0.0);
      Quaternion desiredchestOrientation = new Quaternion(chestOrientation);
      //ReferenceFrame dataframe;
      //ReferenceFrame trajectoryFrame;

      ChestTrajectoryMessage chestPoints = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredchestOrientation, dataframe, trajectoryFrame);

      return chestPoints;
   }
   private PelvisHeightTrajectoryMessage createPelvisZUp(double stepHeight)
   {
      double nominalPelvisHeight;
      MovingReferenceFrame pelvisZUpFrame = drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame();
      FramePoint3D reference = new FramePoint3D(pelvisZUpFrame);

      //createChestTrajectory(ReferenceFrame.getWorldFrame(), pelvisZUpFrame); //calling chest Trajectory method

      //reference.changeFrame(ReferenceFrame.getWorldFrame());
      nominalPelvisHeight = reference.getZ(); //now you have the Z value from world frame perspective

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage();
      pelvisHeightTrajectoryMessage.setEnableUserPelvisControl(true);
      pelvisHeightTrajectoryMessage.setEnableUserPelvisControlDuringWalking(true);
      EuclideanTrajectoryPointMessage waypoint1 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint1.getPosition().setZ(nominalPelvisHeight);
      waypoint1.setTime(2.5);

      EuclideanTrajectoryPointMessage waypoint2 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint2.getPosition().setZ(nominalPelvisHeight+0.1);
      waypoint2.setTime(6.5);

      EuclideanTrajectoryPointMessage waypoint3 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint3.getPosition().setZ(nominalPelvisHeight+ stepHeight + 0.1);
      waypoint3.setTime(17.5);

      waypoint1.getLinearVelocity().setZ(0.0);
      //waypoint2.getLinearVelocity().setZ(0.0);
      waypoint3.getLinearVelocity().setZ(0.0);

      return pelvisHeightTrajectoryMessage;
   }

   private FootstepDataMessage footsteps(int i,RobotSide side, Point3D waypoint, FrameQuaternion quaternion, double swingTime, double transferTime)
   {
      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(side);
      FramePoint3D footPosition = new FramePoint3D(soleFrame);
      FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, waypoint, quaternion);
      footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());
      double pitch = Math.toRadians(10.0);
      double[] zways = {0.1,0.3,0.3};
      double[] xways = {0.85,1.28,1.21};
      SE3TrajectoryPointMessage[] refpoints = new SE3TrajectoryPointMessage[3];
      //outer:
      //for(int j = 0; j <= 1; j++)
      //{

         //SE3TrajectoryPointMessage refpoint = new SE3TrajectoryPointMessage();
         //FrameVector3D linvelocity = new FrameVector3D();
         //linvelocity.set(0.0,0.0,0.0);
         //if(j ==0)
         //{
            //if(i==0){ break outer;}
            if (i > 0)
            {
               if (i == 1) //right
               {
                  footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
                  Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint1.set(xways[i-1] - 0.16, -0.13, zways[i-1]);
                  //quaternion.setYawPitchRoll(0.1, 0.0, 0.0);
                  Point3D waypoint2 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint2.set(xways[i-1] - 0.08, -0.13, zways[i-1]);

                  //refpoint.setTime(1.0);
                  //refpoint.getPosition().set(xways[i - 1] - 0.22, 0.13, zways[i - 1] / 2);
                  //refpoint.getLinearVelocity().set(linvelocity);
                  //refpoint.getOrientation().set(quaternion);
                  //refpoints[i] = refpoint;


                  //footstepDataMessage.getCustomPositionWaypoints().add().set(refpoints.getPosition());
                  //footstepDataMessage.getCustomPositionWaypoints().add().set(refpoints.getOrientation());
                  //MessageTools.copyData(refpoints, footstepDataMessage.getSwingTrajectory());

               }
               else if(i == 2) //left
               {
                  footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
                  //quaternion.setYawPitchRoll(-0.1, 0.0, 0.0);
                  Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint1.set(xways[i-2] - 0.2, 0.13, zways[i-1]+0.1 );
                  Point3D waypoint2 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint2.set(xways[i-1] - 0.02, 0.13, zways[i-1] );

                  //refpoint.setTime(1.0);
                  //refpoint.getPosition().set(xways[i - 1] - 0.22, -0.13, zways[i - 1] / 2);
                  //refpoint.getLinearVelocity().set(linvelocity);
                  //refpoint.getOrientation().set(quaternion);
                  //refpoints[i] = refpoint;

                  //MessageTools.copyData(refpoints, footstepDataMessage.getSwingTrajectory());
               }
               else //right
               {
                  footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
                  //quaternion.setYawPitchRoll(-0.1, 0.0, 0.0);
                  Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint1.set(xways[i-2] - 0.35, -0.13, zways[i-1]+0.1);
                  Point3D waypoint2 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint2.set(xways[i-1] - 0.1, -0.13, zways[i-1]);
               }
               //footstepDataMessage.setSwingDuration(swingTime);
               //footstepDataMessage.setTransferDuration(transferTime);
               //footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());
            }
         //}
         /*else
         {
            if (i > 0)
            {
               if (i % 2 == 0)
               {
                  footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
                  Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint1.set(xways[i - 1] - 0.22, 0.13, zways[i - 1] / 2);
                  quaternion.setYawPitchRoll(0.0, pitch, 0.0);
                  Point3D waypoint2 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint2.set(xways[i - 1] - 0.02, 0.13, 2 * zways[i - 1] / 3);
                  //refpoint.setTime(1.0);
                  //refpoint.getPosition().set(xways[i - 1] - 0.02, 0.13, 2 * zways[i - 1] / 3);
                  //refpoint.getLinearVelocity().set(linvelocity);
                  //refpoint.getOrientation().set(quaternion);
                  //refpoints[i] = refpoint;

                  //footstepDataMessage.getCustomPositionWaypoints().add().set(refpoints.getPosition());
                  //footstepDataMessage.getCustomPositionWaypoints().add().set(refpoints.getOrientation());
                  //MessageTools.copyData(refpoints, footstepDataMessage.getSwingTrajectory());

               }
               else
               {
                  footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
                  Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint1.set(xways[i - 1] - 0.2, -0.13, zways[i - 1] / 3);
                  quaternion.setYawPitchRoll(0.0, pitch, 0.0);
                  Point3D waypoint2 = footstepDataMessage.getCustomPositionWaypoints().add();
                  waypoint2.set(xways[i - 1] - 0.02, -0.13, 2 * zways[i - 1] / 3);
                  //refpoint.setTime(1.0);
                  //refpoint.getPosition().set(xways[i - 1] - 0.02, -0.13, 2 * zways[i - 1] / 3);
                  //refpoint.getLinearVelocity().set(linvelocity);
                  //refpoint.getOrientation().set(quaternion);
                  //refpoints[i] = refpoint;


               }
               //footstepDataMessage.setSwingDuration(swingTime);
               //footstepDataMessage.setTransferDuration(transferTime);
               //footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());

            }*/

            //MessageTools.copyData(refpoints, footstepDataMessage.getSwingTrajectory());
        // }
         //footstepDataMessage.setRobotSide(side.toByte());
         footstepDataMessage.setSwingDuration(swingTime);
         footstepDataMessage.setTransferDuration(transferTime);
         //footstepDataMessage.getLocation().set(footPosition);
         //footstepDataMessage.getOrientation().set(quaternion);
         //MessageTools.copyData(refpoints, footstepDataMessage.getSwingTrajectory());




      //footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());


      return footstepDataMessage;
   }


   private FootstepDataListMessage createFootSteps(DRCRobotModel robotModel,double stepHeight, double swingHeight)
   {
      //create a list of desired footstep
      double stepWidth = 0.13;
      double swingTime = 1.5;
      double defaultTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      //double[][] steps = {{0.6, stepWidth, 0.0}, {0.8, -stepWidth, 0.0}, {1.2, stepWidth, stepHeight}, {1.2, -stepWidth, stepHeight}};//, {1.2, stepWidth, stepHeight}};//,{0.6,stepWidth,0.4},{0.8,-stepWidth,0.8},{0.8,stepWidth,0.8}};

      /*else
      {
         double[][] steps = {{0.3, stepWidth, 0.0}, {0.6, -stepWidth, 0.0}, {0.75, stepWidth, 0.0}, {1.1, -stepWidth, stepHeight}, {1.1, stepWidth, stepHeight}};
         //Point3D[][] steps2 =  {{new Point3D(0.0, 0.0, 0.0)},{0.3,-stepWidth,0.0},{0.6,stepWidth,0.4},{0.8,-stepWidth,0.8},{0.8,stepWidth,0.8}};
      }*/


      ///create a object for the footstepdtatalistmessage class
      //FootstepDataListMessage footstepDataListMessage = HumanoidMessageTools.createFootstepDataListMessage(swingTime, defaultTransferTime);
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      //footstepDataListMessage.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());
      //footstepDataListMessage.setAreFootstepsAdjustable(false); //try changing this later to see the changes

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 4);

      //RobotSide side = RobotSide.LEFT;
      RobotSide tempright = RobotSide.RIGHT;
      MovingReferenceFrame soleZUpFramel = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(robotSides[0]); //left side
      MovingReferenceFrame soleZUpFramer = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(tempright); //right side
      FramePose3D solereferencel = new FramePose3D(soleZUpFramel);
      FramePose3D solereferencer = new FramePose3D(soleZUpFramer);
      solereferencel.changeFrame(ReferenceFrame.getWorldFrame());
      double xsole = solereferencel.getPosition().getX();
      //System.out.println(xsole);
      double ysolel = solereferencel.getPosition().getY();
      double ysoler = solereferencer.getPosition().getY();
      //System.out.println(ysole);
      double zsole = solereferencel.getPosition().getZ();
      //add them to a object of class footstepdatalistmessage
      for (int i = 0 ; i < 4; i++)   //get there in 4 steps - trying making this autonomous in future
      {

         //System.out.println(zsole);
         //System.exit(1);
         /*if(i ==0)
         {
            xsole = 0.6;
            ysole = stepWidth;
         }
         else if (i ==1)
         {
            xsole = 0.8;
            ysole = -stepWidth;
         }
         else if (i == 2)
         {
            xsole = 1.2;
            ysole = stepWidth;
            zsole = stepHeight;
         }
         else
         {
            xsole = 1.2;
            ysole = -stepWidth;
            zsole = stepHeight;
         }*/

         //store the desired step location as Point3D object
         //Point3D point3D = new Point3D(steps[i]);


         //update robot side and variables
         xsole += 0.18;
         if (i>1)
         {
            xsole += 0.25;
         }

         if (i >=2)
         {
            if(i ==3)
            {
               xsole -= 0.5;
            }
            zsole = stepHeight;
         }
         if (i%2 == 0)
         {
            Point3D waypoint = new Point3D(xsole, stepWidth, zsole);
            FrameQuaternion frameQuaternion = new FrameQuaternion();
            //footstepDataMessage.
            //footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, waypoint, frameQuaternion));
            footstepDataListMessage.getFootstepDataList().add().set(footsteps(i,robotSides[i] ,waypoint, frameQuaternion, swingTime, defaultTransferTime));
         }
         else
         {
            Point3D waypoint = new Point3D(xsole, -stepWidth, zsole);
            FrameQuaternion frameQuaternion = new FrameQuaternion();
            //footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(robotSides[i], waypoint, frameQuaternion));
            footstepDataListMessage.getFootstepDataList().add().set(footsteps(i,robotSides[i] ,waypoint, frameQuaternion, swingTime, defaultTransferTime));
         }

         //side = side.getOppositeSide();

         System.out.println(xsole);
         System.out.println(ysoler);
         System.out.println(zsole);

      }

      return footstepDataListMessage;
   }

   private static void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint) // other joint types are - FloatingJoint, FreeJoint, FloatingPlanarJoint, PinJoint, SliderJoint and NullJoints
         pinJoints.add((PinJoint) joint); //type casting

      for(Joint child : joint. getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints); //calling itself to add the child joints too
      }
   }



   private void assertreached(FootstepDataListMessage footsteps)
   {
      int numberofsteps = footsteps.getFootstepDataList().size();
      Point3D lastStep = footsteps.getFootstepDataList().get(numberofsteps-1).getLocation();
      Point3D nextToLastStep = footsteps.getFootstepDataList().get(numberofsteps - 2).getLocation();

      Point3D midStance = new Point3D();
      midStance.interpolate(lastStep, nextToLastStep, 0.5);

      Point3D midpoint = new Point3D(midStance);
      midpoint.addZ(1.0);

      Point3D bounds = new Point3D(0.25, 0.25, 0.3);

      BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(midpoint, bounds);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox3D);
   }
   private DRCRobotModel setTestEnvironment(double stepHeight) throws SimulationExceededMaximumTimeException
   {
    /*      //create environment
      AdjustableStairsEnvironment adjustableStairsEnvironment = new AdjustableStairsEnvironment();
      //adjustableStairsEnvironment.setCourseStartDistance(0.5);
      //adjustableStairsEnvironment.setLandingPlatformParameters(0,0,0,0);
      //adjustableStairsEnvironment.setRailingParameters(0,0,0,0,0,false);
      adjustableStairsEnvironment.setStairsParameters(2,1.0,0.4,0.5);
      adjustableStairsEnvironment.generateTerrains();
*/


      String className = getClass().getSimpleName();
      SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 0.7);
      DRCRobotModel robotModel = getRobotModel();
      //pass this reference to the simulator
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      //drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, DRCRobotModel atlasRobotModel, adjustableStairsEnvironment);
      drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.5, 0.0, 0.0, 0.0)); //setting starting location
      drcSimulationTestHelper.createSimulation(className);
      PushRobotController pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), robotModel.createFullRobotModel().getChest().getParentJoint().getName(), new Vector3D(0.0, 0.0, 0.15));


      setUpCamera();
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);
      //createTorqueGraphs(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));
      //printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());
      assertTrue(success);

      return robotModel;
   }

   private void setUpCamera()
   {
      Point3D cameraFix = new Point3D(0.0,0.0,0.89);
      Point3D cameraPosition = new Point3D(10.0,2.0,1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
/*
   private void getPinJoints(Robot robot, List<PinJoint> pinJoint) //call the recursivelyAddPinJoints to get a list of all PinJoint(only) and ignore the other types of joints
   {
      for(Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoint);
      }
   }
   private void createTorqueGraphs(SimulationConstructionSet scs, Robot robot)
   {

      List<PinJoint> pinJoints = new ArrayList<>();

      getPinJoints(robot, pinJoints);

      List<String> torsojoints = new ArrayList<String>();
      List<String> leftlegjoints = new ArrayList<String>();
      List<String> rightlegjoints = new ArrayList<String>();

      for (PinJoint joint : pinJoints)
      {
         String name = joint.getTauYoVariable().getName();

         if (name.contains("l_leg"))
         {
            leftlegjoints.add(name);
         }
         else if (name.contains("r_leg"))
         {
            rightlegjoints.add(name);
         }
         else if (name.contains("back"))
         {
            torsojoints.add(name);
         }

         addGraph(scs, torsojoints);
         addGraph(scs, leftlegjoints);
         addGraph(scs, rightlegjoints);
      }
   }

      private void addGraph(SimulationConstructionSet scs, List<String> joint)
      {
         //scs.setupGraph(joint.toArray(new String[0])); //casting it to string object so that it can be passed to plot graph
      }

      private void printMinMax(SimulationConstructionSet scs)
      {
         StandardSimulationGUI window = scs.getGUI();
         GraphArrayPanel panel = window.getGraphArrayPanel();
         ArrayList<YoGraph> graphs = panel.getGraphsOnThisPanel();

         if (graphs.size() != 0)
         {
            return;
         }

         for(YoGraph graph: graphs)
         {
            ArrayList<DataEntry> entries = graph. getEntriesOnThisGraph();
            entries.forEach(entry -> System.out.println(entry.getVariableName() + "Max Torque: [" + Math.max(Math.abs(entry.getMin()), Math.abs(entry.getMax()))+ "]"));
         }
      }
*/
}

