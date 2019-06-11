package us.ihmc.avatar.stepUptest;

import static us.ihmc.robotics.Assert.*;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.*;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.factory.*;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.TrajectoryStatusMessageHelper.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.packets.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.*;
import us.ihmc.euclid.transform.*;
import us.ihmc.euclid.tuple2D.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.euclid.tuple4D.interfaces.*;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.flatGroundPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.aStar.*;
import us.ihmc.footstepPlanning.graphSearch.aStar.AStarBestEffortTest.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.footstepPlanning.graphSearch.planners.*;
import us.ihmc.footstepPlanning.simplePlanners.*;
import us.ihmc.footstepPlanning.tools.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.footstep.*;
import us.ihmc.idl.IDLSequence.Double;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.*;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.*;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.kinematics.*;
import us.ihmc.robotics.math.trajectories.trajectorypoints.*;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.*;
import us.ihmc.robotics.partNames.*;
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
import us.ihmc.yoVariables.registry.*;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.util.*;
import java.util.List;

public abstract class AvatarStepUp implements MultiRobotTestInterface
{
   public us.ihmc.euclid.tuple3D.Point3D waypointtotransfer;
   //private AvatarStepUp planner;
   private YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
   private AStarFootstepPlanner planner;
   private YoBoolean walkPaused;
   private AvatarSimulation avatarSimulation;

   private int numberOfFootstepByPlanner;
   private double stepLength = 0.7;
   private double stepWidth = 0.13;
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private final AtlasRobotModel robotModel = new AtlasRobotModel(version,RobotTarget.SCS,false);
   private final AtlasJointMap jointMap = new AtlasJointMap(version,robotModel.getPhysicalProperties());//cannot use getRobotModel its of type DRCRobotModel while AtlasJintMap requires AtlasRobotModel object
   private ArmJointName[] armJoint = getArmJointNames();
   private Random random = new Random(42);
   //private DRCRobotModel robotModela = getRobotModel();
   private FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();




   private DRCSimulationTestHelper drcSimulationTestHelper;
   //private final AtlasR

   @BeforeEach
   public  void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @BeforeEach
   public void setup()
   {
      FootstepPlannerParameters parameters = new BestEffortPlannerParameters(3);
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      this.planner = AStarFootstepPlanner.createPlanner(parameters, null, footPolygons, expansion, registry);
      planner.setTimeout(0.5);
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

   @AfterEach
   public void tearDown()
   {
      planner = null;
      ReferenceFrameTools.clearWorldFrameTree();
   }

   protected ArmJointName[] getArmJointNames()
   {
      return jointMap.getArmJointNames();
   }

   protected int getArmTrajectoryPoints()
   {
      return 4;
   }

   protected int getArmDoF()
   {
      return 6;
   }


   //start writing your step up code here now
   @Test
   public void stepUpSmall() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.3;
      double swingHeight = 0.10; //maybe needs to be changed
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
      //ChestTrajectoryMessage chestTrajectoryPoints1 = createChestTrajectory(ReferenceFrame.getWorldFrame(),drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame()); //create desired chest trajectory

      ArmTrajectoryMessage rightArmTrajectoryMessages = createArmRightTrajectory();
      ArmTrajectoryMessage leftArmTrajectoryMessages = createArmLeftTrajectory();
      //System.out.println(armJoint);
      //System.exit(1);
      //createFootstepUsingFootstepPlanner();


      drcSimulationTestHelper.publishToController(footsteps);
      //pausing the walking action for a moment
      //walkPaused.set(true);

      drcSimulationTestHelper.publishToController(pelvisDHeight);
      //drcSimulationTestHelper.publishToController(chestTrajectoryPoints1);
      createAndPublishChestTrajectory(ReferenceFrame.getWorldFrame(),drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame());

      //pausing the footstep comment to get the chest and pelvis at appropriate orientation
      /*
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.5);

      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      drcSimulationTestHelper.publishToController(pauseWalkingMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      PauseWalkingMessage resumeWalkingMessag = HumanoidMessageTools.createPauseWalkingMessage(false);
      drcSimulationTestHelper.publishToController(resumeWalkingMessag);*/


      //drcSimulationTestHelper.publishToController(leftArmTrajectoryMessages);
      //drcSimulationTestHelper.publishToController(rightArmTrajectoryMessages);
      //ThreadTools.sleep(2000l);
      //ChestTrajectoryMessage chestTrajectoryPoints2 = createChestTrajectory(ReferenceFrame.getWorldFrame(),drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame(), 1.5,0.0);
      //drcSimulationTestHelper.publishToController(chestTrajectoryPoints2);
      //createFootstepUsingFootstepPlanner(stepLength,stepWidth);
      /*
      int i = 0; //variable for iterating over eac arm orientation message (an array of double)
      do
      {

         drcSimulationTestHelper.publishToController(armTrajectoryMessages[i]);
         ++i;
      }
      while (i<2);*/



      //Object<Point3D> waypoints = footsteps.getFootstepDataList().get(1).getCustomPositionWaypoints();
      //Double Proportion = footsteps.getFootstepDataList().get(1).getCustomWaypointProportions();
      //System.out.println(waypoints);
      //System.out.println(Proportion);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTouchdownTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      // robot fell
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime +2.0*initialFinalTransfer + 12.0));

      // robot did not fall but did not reach goal
      assertreached(footsteps);

      ThreadTools.sleepForever(); //does not kill the simulation
   }

   private ArmTrajectoryMessage createArmLeftTrajectory()
   {
      /*
      ArmTrajectoryMessage[] armPoses = new ArmTrajectoryMessage[]{new ArmTrajectoryMessage(), new ArmTrajectoryMessage()};
      double trajectoryTime = 0.5;
      //double[][] armJoints = new double[][] {{-2.5, -0.8, 1.12, 1.48, 0.9, -1.0, -2.8},{0.44, 0.02, 2.7, -1.78, 0.3, -1.5, -0.5}};
      double[][] armJoints = new double[2][getArmTrajectoryPoints()];
      for(int armJointindex = 0 ; armJointindex < getArmDoF(); ++armJointindex)
      {
       for (int trajectoryPointIndex =0; trajectoryPointIndex < getArmTrajectoryPoints(); ++trajectoryPointIndex)
       {

       }
      }
      RobotSide[] side = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 2);
      // this message commands the controller to move an arm in JOINTSPACE to the desired joint angles while going through the specified trajectory points
      for(int j=0;j < 2 ; ++j)
      {
         double[] desiredArmPose = armJoints[j];
         ArmTrajectoryMessage armPose = HumanoidMessageTools.createArmTrajectoryMessage(side[j], trajectoryTime, desiredArmPose);
         armPoses[j] = armPose;
      }
      return armPoses;*/
      //OneDoFJointTrajectoryMessage[] leftandrightinfo = new OneDoFJointTrajectoryMessage[] {new OneDoFJointTrajectoryMessage(), new OneDoFJointTrajectoryMessage()}

      ArrayList<OneDoFJointTrajectoryMessage> leftandrightinfo = new ArrayList<>();

      ArmTrajectoryMessage leftHandMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT);
      //ArmTrajectoryMessage rightHandMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT);

      ArmJointName[] armJointName = getArmJointNames();

      ArrayList<OneDoFJointTrajectoryMessage> leftArmTrajectory = new ArrayList<>();
      //ArrayList<OneDoFJointTrajectoryMessage> rightArmTrajectory = new ArrayList<>();

      for(int armJointindex = 0 ; armJointindex < getArmDoF(); ++armJointindex)
      {
         OneDoFJointTrajectoryMessage leftJointTrajectory = new OneDoFJointTrajectoryMessage();
         //OneDoFJointTrajectoryMessage rightJointTrajectory = new OneDoFJointTrajectoryMessage();

         for (int trajectoryPointIndex =0; trajectoryPointIndex < getArmTrajectoryPoints(); ++trajectoryPointIndex)
         {
            leftJointTrajectory.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage((double) (2*trajectoryPointIndex +1), getRandomJointAngle(RobotSide.LEFT,armJoint[armJointindex],fullRobotModel),(double) 0));
            //rightJointTrajectory.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage((double) (2*trajectoryPointIndex +1), getRandomJointAngle(RobotSide.RIGHT, armJoint[armJointindex], fullRobotModel), (double) 0));
         }
         leftHandMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(leftJointTrajectory);
         //rightHandMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(rightJointTrajectory);
         leftArmTrajectory.add(leftJointTrajectory);
         //rightArmTrajectory.add(rightJointTrajectory);
      }
      //leftandrightinfo[0] = leftArmTrajectory;
      return leftHandMessage;
   }

   private ArmTrajectoryMessage createArmRightTrajectory()
   {
      /*
      ArmTrajectoryMessage[] armPoses = new ArmTrajectoryMessage[]{new ArmTrajectoryMessage(), new ArmTrajectoryMessage()};
      double trajectoryTime = 0.5;
      //double[][] armJoints = new double[][] {{-2.5, -0.8, 1.12, 1.48, 0.9, -1.0, -2.8},{0.44, 0.02, 2.7, -1.78, 0.3, -1.5, -0.5}};
      double[][] armJoints = new double[2][getArmTrajectoryPoints()];
      for(int armJointindex = 0 ; armJointindex < getArmDoF(); ++armJointindex)
      {
       for (int trajectoryPointIndex =0; trajectoryPointIndex < getArmTrajectoryPoints(); ++trajectoryPointIndex)
       {

       }
      }
      RobotSide[] side = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 2);
      // this message commands the controller to move an arm in JOINTSPACE to the desired joint angles while going through the specified trajectory points
      for(int j=0;j < 2 ; ++j)
      {
         double[] desiredArmPose = armJoints[j];
         ArmTrajectoryMessage armPose = HumanoidMessageTools.createArmTrajectoryMessage(side[j], trajectoryTime, desiredArmPose);
         armPoses[j] = armPose;
      }
      return armPoses;*/
      //OneDoFJointTrajectoryMessage[] leftandrightinfo = new OneDoFJointTrajectoryMessage[] {new OneDoFJointTrajectoryMessage(), new OneDoFJointTrajectoryMessage()}

      //ArrayList<OneDoFJointTrajectoryMessage> leftandrightinfo = new ArrayList<>();

      //ArmTrajectoryMessage leftHandMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT);
      ArmTrajectoryMessage rightHandMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT);

      //ArmJointName[] armJointName = getArmJointNames();

      //ArrayList<OneDoFJointTrajectoryMessage> leftArmTrajectory = new ArrayList<>();
      ArrayList<OneDoFJointTrajectoryMessage> rightArmTrajectory = new ArrayList<>();

      for(int armJointindex = 0 ; armJointindex < getArmDoF(); ++armJointindex)
      {
         //OneDoFJointTrajectoryMessage leftJointTrajectory = new OneDoFJointTrajectoryMessage();
         OneDoFJointTrajectoryMessage rightJointTrajectory = new OneDoFJointTrajectoryMessage();

         for (int trajectoryPointIndex =0; trajectoryPointIndex < getArmTrajectoryPoints(); ++trajectoryPointIndex)
         {
            //leftJointTrajectory.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage((double) (2*trajectoryPointIndex +1), getRandomJointAngle(RobotSide.LEFT,armJoint[armJointindex],fullRobotModel),(double) 0));
            rightJointTrajectory.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage((double) (2*trajectoryPointIndex +1), getRandomJointAngle(RobotSide.RIGHT, armJoint[armJointindex], fullRobotModel), (double) 0));
         }
         //leftHandMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(leftJointTrajectory);
         rightHandMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(rightJointTrajectory);
         //leftArmTrajectory.add(leftJointTrajectory);
         rightArmTrajectory.add(rightJointTrajectory);
      }
      //leftandrightinfo[0] = leftArmTrajectory;
      return rightHandMessage;
   }


   /*private ChestTrajectoryMessage createChestTrajectory(ReferenceFrame dataframe, ReferenceFrame trajectoryFrame)
   {
      double trajectoryTime = 5.5;
      FrameQuaternion chestOrientation1 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation2 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      //chestOrientation.appendYawRotation(2.0); //there also these append methods that you can use to mention only yaw,roll or pitch angles.
      double leanAngle = 20.0; //original values 20.0 and yaw was -2.36
      chestOrientation1.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.00, Math.toRadians(leanAngle), 0.0);
      //Quaternion desiredchestOrientation = new Quaternion(chestOrientation);
      chestOrientation2.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), -2.36, 0.0, 0.0);

      FrameQuaternion[] desiredChestOrientations = {chestOrientation1, chestOrientation2};

      //ReferenceFrame dataframe;
      //ReferenceFrame trajectoryFrame;
      FrameSO3TrajectoryPointList trajectoryPointList = new FrameSO3TrajectoryPointList();

      //ChestTrajectoryMessage chestPoints = HumanoidMessageTools
      // .createChestTrajectoryMessage(trajectoryTime, desiredchestOrientations[], dataframe, trajectoryFrame);
      ChestTrajectoryMessage chestPoint = new ChestTrajectoryMessage();

      //FrameSO3TrajectoryPoint ksjabdhk = chestPoint.getSo3Trajectory().getTaskspaceTrajectoryPoints().

      //SO3TrajectoryPoint waypoint1 = new SO3TrajectoryPoint();
      Vector3D zeroAngularVelocity = new Vector3D();
      SO3TrajectoryPointMessage wayPoint1 = chestPoint.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
      //SO3TrajectoryMessage waypint1 = chestPoint.getSo3Trajectory();
      //chestPoint.getSo3Trajectory().set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime, desiredChestOrientations[1],trajectoryFrame));
      //chestPoint.getSo3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(trajectoryTime, desiredChestOrientations[1], zeroAngularVelocity));
      wayPoint1.set(HumanoidMessageTools.createSO3TrajectoryPointMessage(trajectoryTime, desiredChestOrientations[0], zeroAngularVelocity));
      //!!!trajectoryPointList.addTrajectoryPoint(trajectoryTime, desiredChestOrientations[0], zeroAngularVelocity);

      //wayPoint1.setTime();
      wayPoint1.setSequenceId(chestPoint.getSequenceId());
      System.out.println("ID1: " + wayPoint1.getSequenceId());
      //chestPoint.setSequenceId(chestPoint.getSequenceId());
      //chestPoint.getSo3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(trajectoryTime, desiredChestOrientations[1], zeroAngularVelocity));
      //waypoint1.getOrientation().set(desiredChestOrientations[0]);
      //waypoint1.setTime(3.0);
      //SO3TrajectoryPointMessage waypoint2 = chestPoint.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
      //trajectoryPointList.addTrajectoryPoint(trajectoryTime, desiredChestOrientations[1], zeroAngularVelocity);
      //waypoint2.getOrientation().set(desiredChestOrientations[1]);
      //waypoint2.setTime(3.0);
      wayPoint1.set(HumanoidMessageTools.createSO3TrajectoryPointMessage(trajectoryTime, desiredChestOrientations[1], zeroAngularVelocity));
      wayPoint1.setSequenceId(chestPoint.getSequenceId());
      System.out.println("ID2: " + wayPoint1.getSequenceId());
      //chestPoint.setSequenceId(chestPoint.getSequenceId());

      //return chestPoints;
      return chestPoint;
   }*/

   private FootstepDataMessage[] createFootstepUsingFootstepPlanner(double stepLength, double stepWidth)
   {

      //AStarBestEffortTest Aplanner = new AStarBestEffortTest();
      //AStarFootstepPlanner planner = A
      //Aplanner.setup();


      /*
      ConvexPolygon2D groundPlane = new ConvexPolygon2D();
      groundPlane.addVertex(-1.0,-1.0);
      groundPlane.addVertex(-1.0,1.0);
      groundPlane.addVertex(1.0,-1.0);
      groundPlane.addVertex(1.0,1.1);

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), groundPlane);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);

      */

      Point3D initialWaypoint = waypointtotransfer; //transfer the previous waypoint to a local variable
      //Point3D goleposition = new Point3D((1+2.5*stepLength) -0.3,stepWidth,0.0);
      Point2D initialStance = new Point2D(initialWaypoint.getX(), initialWaypoint.getY());
      FramePose2D initiaWaypoint2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStance, 0.0);

      double finalXVale = (1+2.5*stepLength) +0.2;
      Point2D finalStance = new Point2D( finalXVale,stepWidth);
      FramePose2D finalWaypoint2d = new FramePose2D(ReferenceFrame.getWorldFrame(), finalStance, 0.0);
      //FramePoint3D initialWaypoint3d = FlatGroundPlanningUtils.poseFormPose2d()
      //FramePose3D framePose3D = new FramePose3D(ReferenceFrame.getWorldFrame(), waypoint);
      RobotSide initialSide = RobotSide.LEFT;
      // starting foot position for left
      FramePose3D initialWaypoint3d = FlatGroundPlanningUtils.poseFormPose2d(initiaWaypoint2d, 0.0);
      FramePose3D finalWaypoint3d = FlatGroundPlanningUtils.poseFormPose2d(finalWaypoint2d,0.0);
      SimpleFootstep simpleFootstep = new SimpleFootstep(RobotSide.LEFT, finalWaypoint3d);
      //FootstepPlan footsteps = PlannerTools.runPlanner(gerPlanner());
      //DepthFirstFootstepPlanner plan;
      //DepthFirstFootstepPlannerOnFlatTest planner = new DepthFirstFootstepPlannerOnFlatTest();
      //planner.set

      //planner.getPlanner();
      //planner.getPlanner().
      //planner.testJustStraightLine();
      //FootstepPlannerGoal goal = new FootstepPlannerGoal();
      //goal.setSingleFootstepGoal(simpleFootstep);
      //planner.getPlanner().setInitialStanceFoot(initialWaypoint3d, RobotSide.LEFT);
      //planner.getPlanner().setGoal(goal);
      //plan.setInitialStanceFoot(initialWaypoint3d, RobotSide.LEFT);
      //PlannerTools plannerTools = new PlannerTools();
      FootstepPlan footstep = PlannerTools.runPlanner(planner, initialWaypoint3d, initialSide, finalWaypoint3d,null, true);
     // footStepPlan.getNumberOfSteps();
     // FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();


      numberOfFootstepByPlanner = footstep.getNumberOfSteps();
      FootstepDataMessage[] footsteps = new FootstepDataMessage[numberOfFootstepByPlanner];
      RobotSide side = RobotSide.LEFT;
      for (int footstepIndex = 0; footstepIndex < footstep.getNumberOfSteps(); footstepIndex++)
      {
         //System.out.println(footstep.getFootstep(footstepIndex).getSoleFramePose().getPosition().getX() +" " +  footstep.getFootstep(footstepIndex).getSoleFramePose().getPosition().getY() +" " + footstep.getFootstep(footstepIndex).getSoleFramePose().getPosition().getZ()+ "\n");

         footstep.getFootstep(footstepIndex).getSoleFramePose().getPosition().getX();

         Point3D waypoint = new Point3D(footstep.getFootstep(footstepIndex).getSoleFramePose().getPosition().getX(), stepWidth, 0.0);
         FrameQuaternion orientation = new FrameQuaternion();
         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, waypoint, orientation);
         footsteps[footstepIndex] = footstepDataMessage;
         side = side.getOppositeSide();
         stepWidth = -stepWidth;

         //footstepDataListMessage.getFootstepDataList().add().set(footstepDataMessage);
      }

      //drcSimulationTestHelper.publishToController(footstepDataListMessage);

      // goal pose just before the walls region
      return footsteps;
   }

   private void createAndPublishChestTrajectory(ReferenceFrame dataframe, ReferenceFrame trajectoryFrame)
   {
      double trajectoryTime = 10.5;
      FrameQuaternion chestOrientation1 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation2 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation3 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation4 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      //chestOrientation.appendYawRotation(2.0); //there also these append methods that you can use to mention only yaw,roll or pitch angles.
      double leanAngle = 20.0; //original values 20.0 and yaw was -2.36
      chestOrientation1.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.00, Math.toRadians(leanAngle), 0.0);
      //Quaternion desiredchestOrientation = new Quaternion(chestOrientation);
      chestOrientation2.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), -2.36, 0.0, 0.0);
      chestOrientation3.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.2);
      chestOrientation4.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FrameQuaternion[] desiredChestOrientations = {chestOrientation1, chestOrientation2, chestOrientation3,chestOrientation4};

      //ChestTrajectoryMessage chestPoint = new ChestTrajectoryMessage();
      //executes this one first then the goes in reverse starting from the ver last one
      ChestTrajectoryMessage bend = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredChestOrientations[0], dataframe, trajectoryFrame);
      drcSimulationTestHelper.publishToController(bend);

      ChestTrajectoryMessage straight = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime + 5.5, desiredChestOrientations[3], dataframe, trajectoryFrame);
      straight.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      straight.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      //bend.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(yaw.getSequenceId());
      drcSimulationTestHelper.publishToController(straight);

      /*

      ChestTrajectoryMessage yaw = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredChestOrientations[1], dataframe, trajectoryFrame);
      yaw.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      yaw.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      //bend.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(yaw.getSequenceId());
      drcSimulationTestHelper.publishToController(yaw);

      ChestTrajectoryMessage roll = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredChestOrientations[2], dataframe, trajectoryFrame);
      roll.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      roll  .getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      //bend.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(yaw.getSequenceId());
      drcSimulationTestHelper.publishToController(roll);
      /*
      ChestTrajectoryMessage straight1 = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredChestOrientations[3], dataframe, trajectoryFrame);
      straight1.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      straight1.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      //bend.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(yaw.getSequenceId());
      drcSimulationTestHelper.publishToController(straight1);

      System.out.println("yaw ID: " + bend.getSequenceId());
      System.out.println("bend ID:" + bend.getSequenceId());
      System.out.println("yaw1 ID: " + roll.getSequenceId());
      System.out.println("yaw2 ID: " + straight.getSequenceId());*/
   }

   private PelvisHeightTrajectoryMessage createPelvisZUp(double stepHeight)
   {
      double nominalPelvisHeight;
      MovingReferenceFrame pelvisZUpFrame = drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame();
      FramePoint3D reference = new FramePoint3D(pelvisZUpFrame);

      //createChestTrajectory(ReferenceFrame.getWorldFrame(), pelvisZUpFrame); //calling chest Trajectory method

      reference.changeFrame(ReferenceFrame.getWorldFrame());
      nominalPelvisHeight = reference.getZ(); //now you have the Z value from world frame perspective

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage();
      pelvisHeightTrajectoryMessage.setEnableUserPelvisControl(true);
      pelvisHeightTrajectoryMessage.setEnableUserPelvisControlDuringWalking(true);
      /*
      EuclideanTrajectoryPointMessage waypoint1 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint1.getPosition().setZ(0.0*nominalPelvisHeight);
      waypoint1.setTime(2.5);

      EuclideanTrajectoryPointMessage waypoint2 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint2.getPosition().setZ((1.0*nominalPelvisHeight)+0.1);
      waypoint2.setTime(5.0);

      EuclideanTrajectoryPointMessage waypoint3 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint3.getPosition().setZ(nominalPelvisHeight+ stepHeight);
      waypoint3.setTime(6.5);

      waypoint1.getLinearVelocity().setZ(0.0);
      //waypoint2.getLinearVelocity().setZ(0.0);
      waypoint3.getLinearVelocity().setZ(0.0);
      System.out.println("Pelvis Height of robot from worldFrame perspective:" + nominalPelvisHeight);
      */

      EuclideanTrajectoryPointMessage waypoint1 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint1.getPosition().setZ(nominalPelvisHeight);
      waypoint1.setTime(9.5);

      EuclideanTrajectoryPointMessage waypoint2 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint2.getPosition().setZ(0.2*nominalPelvisHeight);
      waypoint2.setTime(12.5);

      EuclideanTrajectoryPointMessage waypoint3 = pelvisHeightTrajectoryMessage.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().add();
      waypoint3.getPosition().setZ(nominalPelvisHeight);
      waypoint3.setTime(17.5);

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
      double[] xways = {0.85,1.28,1.31};
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
      double swingTime = 2.0;
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
               xsole -= 0.4;
            }
            zsole = stepHeight;
         }
         if (i%2 == 0) //left
         {
            Point3D waypoint = new Point3D(xsole, stepWidth, zsole);
            if(i==2) {waypointtotransfer = waypoint;}
            FrameQuaternion frameQuaternion = new FrameQuaternion();
            //footstepDataMessage.
            //footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, waypoint, frameQuaternion));
            footstepDataListMessage.getFootstepDataList().add().set(footsteps(i,robotSides[i] ,waypoint, frameQuaternion, swingTime, defaultTransferTime));

         }
         else //right
         {
            Point3D waypoint = new Point3D(xsole, -stepWidth, zsole);
            FrameQuaternion frameQuaternion = new FrameQuaternion();
            //footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(robotSides[i], waypoint, frameQuaternion));
            footstepDataListMessage.getFootstepDataList().add().set(footsteps(i,robotSides[i] ,waypoint, frameQuaternion, swingTime, defaultTransferTime));
         }
         //side = side.getOppositeSide();
         System.out.println("x position sole: " + xsole);
         System.out.println("yr position sole: " + ysoler);
         System.out.println("z position sole: "+ zsole);
      }

      FootstepDataMessage[] toBeAddedFootsteps = createFootstepUsingFootstepPlanner(stepLength,stepWidth);
      for(int footstepIndex = 0; footstepIndex < toBeAddedFootsteps.length; footstepIndex++)
      {

         if (footstepIndex == 1)
         {
            //toBeAddedFootsteps[footstepIndex].getLocation().getX();
            //FootstepDataMessage modifiedPoint = new FootstepDataMessage();
            Point3D newpoint = new Point3D(toBeAddedFootsteps[footstepIndex].getLocation().getX() -0.3,toBeAddedFootsteps[footstepIndex].getLocation().getY() , toBeAddedFootsteps[footstepIndex].getLocation().getZ());
            toBeAddedFootsteps[footstepIndex].getLocation().set(newpoint);
            footstepDataListMessage.getFootstepDataList().add().set(toBeAddedFootsteps[footstepIndex]);
         }
         else
         {footstepDataListMessage.getFootstepDataList().add().set(toBeAddedFootsteps[footstepIndex]);}
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

   protected double getRandomJointAngle(RobotSide side, ArmJointName armJointName, FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      OneDoFJointBasics armJoint = fullHumanoidRobotModel.getArmJoint(side, armJointName);
      if (armJoint!= null)
      {
         double jointAngle = armJoint.getJointLimitLower() + (armJoint.getJointLimitUpper() - armJoint.getJointLimitLower()) * random.nextDouble();
         return jointAngle;
      }
      else
      {
         return 0.0;
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

      Point3D bounds = new Point3D(0.25, 0.25, 1.0);

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
      //SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 0.7);
      Wallswithstairs wall = new Wallswithstairs(0.5, 1.5, stepHeight);

      DRCRobotModel robotModel = getRobotModel();

      //pass this reference to the simulator
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, wall);

      //drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, DRCRobotModel atlasRobotModel, adjustableStairsEnvironment);
      drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.5, 0.0, 0.0, 0.0)); //setting starting location
      drcSimulationTestHelper.createSimulation(className);
      //PushRobotController pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), robotModel.createFullRobotModel().getChest().getParentJoint().getName(), new Vector3D(0.0, 0.0, 0.15));


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

   private class BestEffortPlannerParameters extends DefaultFootstepPlanningParameters
   {
      private final int minimumStepsForBestEffortPlan;

      BestEffortPlannerParameters(int minimumStepsForBestEffortPlan)
      {
         this.minimumStepsForBestEffortPlan = minimumStepsForBestEffortPlan;
      }

      @Override
      public boolean getReturnBestEffortPlan()
      {
         return true;
      }

      @Override
      public int getMinimumStepsForBestEffortPlan(){return minimumStepsForBestEffortPlan;}
   }
   //public us.ihmc.euclid.tuple3D.Point3D getWaypoint()
   //@Override
   //
   /*
   public FootstepPlanner getPlanner()
   {
      return planner;
   }*/

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



