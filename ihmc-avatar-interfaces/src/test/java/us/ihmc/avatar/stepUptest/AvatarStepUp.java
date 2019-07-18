package us.ihmc.avatar.stepUptest;

import static us.ihmc.robotics.Assert.*;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.*;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.factory.*;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.stepUptest.AvatarStepUp.*;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.communication.packets.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.*;
import us.ihmc.euclid.tuple2D.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.footstepPlanning.graphSearch.planners.*;
import us.ihmc.footstepPlanning.simplePlanners.*;
import us.ihmc.footstepPlanning.tools.*;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.humanoidBehaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.dispatcher.*;
import us.ihmc.humanoidBehaviors.utilities.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.*;
import us.ihmc.humanoidRobotics.communication.subscribers.*;
import us.ihmc.humanoidRobotics.footstep.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.pubsub.DomainFactory.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.robotDataLogger.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.sensors.*;
import us.ihmc.robotics.trajectories.*;
import us.ihmc.ros2.*;
import us.ihmc.sensorProcessing.parameters.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.*;
import us.ihmc.simulationConstructionSetTools.util.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;
import us.ihmc.simulationconstructionset.util.simulationTesting.*;

import us.ihmc.tools.*;
import us.ihmc.yoVariables.registry.*;
import us.ihmc.yoVariables.variable.*;

import java.util.*;
import java.util.List;

public abstract class AvatarStepUp implements MultiRobotTestInterface
{
   public enum StartingLocation
   {
      START_FROM_SCRATCH,
      START_FROM_FIDUCIAL
   }


   private final boolean DEBUG = true;
   private final boolean step_up_door = true;
   private final boolean Walls_with_stairs = false;
   private int tmpcounter = 0;

   public us.ihmc.euclid.tuple3D.Point3D waypointtotransfer;

   private YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
   private AStarFootstepPlanner planner;

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
   private FullHumanoidRobotModel fullRobotModel;
   private final double stepHeight = 0.3;

   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private StepUpDoor stepUpDoor =new StepUpDoor(0.5,1.7,stepHeight,yoGraphicsListRegistry);;
   private boolean IS_PAUSING_ON;// = false;  //should be false for now
   private boolean IS_CHEST_ON;// = false;   //reset to final after testing
   private boolean IS_LEFTARM_ON;// = false;
   private boolean IS_RIGHTARM_ON;// = false;
   private boolean IS_PELVIS_ON;// = true;
   private boolean IS_FOOTSTEP_ON;// = true;
   private Pose3D startingPoint = new Pose3D(0.5,0.0,0.0,0.0,0.0,0.0);
   private Pose3D startingFromFiducial = new Pose3D(StepUpDoor.getFiducialPosition().getX(),0.5,0.0,0.0,0.0,0.0);

   private OffsetAndYawRobotInitialSetup pos1 = new OffsetAndYawRobotInitialSetup(startingPoint.getX(),startingPoint.getY(),startingPoint.getZ(),startingPoint.getYaw());
   private OffsetAndYawRobotInitialSetup pos2 = new OffsetAndYawRobotInitialSetup(startingFromFiducial.getX(),startingFromFiducial.getY(),startingFromFiducial.getZ(),startingFromFiducial.getYaw());
   private OffsetAndYawRobotInitialSetup startingPos;

   private final boolean startWithoutBeingTriggeredFromMessagePacket = true;


   private BehaviorDispatcher<HumanoidBehaviorType> behaviorDispatcher;
   private Ros2Node ros2Node;
   private HumanoidFloatingRootJointRobot robot;
   private YoDouble yoTime;
   private HumanoidRobotDataReceiver robotDataReceiver;

   private HumanoidReferenceFrames referenceFrames;
   private YoBoolean yoDoubleSupport;
   private AtlasPrimitiveActions atlasPrimitiveActions;
   private WalkingStatusMessage newStatusReference = new WalkingStatusMessage();

   private StartingLocation startFromHere; //look at how to initialize this before the BeforeEach annotations




   private DRCSimulationTestHelper drcSimulationTestHelper;

   public final HashMap<String, OffsetAndYawRobotInitialSetup> initalLocation = new HashMap<>();



   //private final AtlasR
   //before each sequence is in accordance with their appearance
   @BeforeEach
   public  void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

   }



   @BeforeEach
   public void setup()
   {

      addMapping(StartingLocation.START_FROM_SCRATCH,pos1);
      addMapping(StartingLocation.START_FROM_FIDUCIAL,pos2);
      FootstepPlannerParameters parameters = new BestEffortPlannerParameters(3);
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      this.planner = AStarFootstepPlanner.createPlanner(parameters, null, footPolygons, expansion, registry);
      planner.setTimeout(0.5);

      String className = getClass().getSimpleName();


      if(Walls_with_stairs)
      {
         Wallswithstairs wall = new Wallswithstairs(0.5, 1.7, stepHeight);
         drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, wall);
      }

//      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      if(step_up_door)
      {
//         stepUpDoor = new StepUpDoor(0.5,1.7,stepHeight,yoGraphicsListRegistry);
//         yoGraphicsListRegistry.registerYoGraphicsList(stepUpDoor.getSphereRobot().getYoGraphicsList());
         drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, stepUpDoor);
      }

      /** This is where you change the starting position. Tried writing a set method being called in Atlas class but the trigger code structure does allow setting before simulation initialisation this leading to null pointer exception **/
      startFromHere = StartingLocation.START_FROM_FIDUCIAL;


      DRCRobotModel robotModel = getRobotModel();
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(version,RobotTarget.SCS, false);

//      drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.5, 0.0, 0.0, 0.0)); //setting starting location
//      drcSimulationTestHelper.setStartingLocation(getStartingLocationOffset(startFromHere));
//
//      drcSimulationTestHelper.setStartingLocation(getStartingLocationOffset(StartingLocation.START_FROM_FIDUCIAL));

      drcSimulationTestHelper.setStartingLocation(setStartingLocationOffset(startFromHere));
      drcSimulationTestHelper.createSimulation(className);

      registry = new YoVariableRegistry(getClass().getSimpleName());
      this.yoTime = new YoDouble("yoTime",registry);

      this.ros2Node = new ROS2Tools().createRos2Node(PubSubImplementation.INTRAPROCESS, "Avatar_step_up");
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           WalkingStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkAndPublishChestTrajectoryMessage);
      //robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();


      behaviorDispatcher = setupBehaviorDispatcher(getRobotModel().getSimpleRobotName(), fullRobotModel, ros2Node, yoGraphicsListRegistry, registry);

      CapturePointUpdatable capturePointUpdatable = createCapturePointUpdateable(drcSimulationTestHelper, registry, yoGraphicsListRegistry);
      behaviorDispatcher.addUpdatable(capturePointUpdatable);

      yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();



      //WholeBodyControllerParameters wholeBodyControllerParameters;


      HumanoidRobotSensorInformation sensorInformation = getRobotModel().getSensorInformation();
      for(RobotSide robotSide : RobotSide.values())
      {
         WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(drcSimulationTestHelper.getRobotName(), robotSide, fullRobotModel, sensorInformation, robotDataReceiver.getForceSensorDataHolder(), IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT, drcSimulationTestHelper.getRos2Node(), registry);

         behaviorDispatcher.addUpdatable(wristSensorUpdatable);
      }

      referenceFrames = robotDataReceiver.getReferenceFrames();

      atlasPrimitiveActions = new AtlasPrimitiveActions(getSimpleRobotName(), ros2Node, getRobotModel().getFootstepPlannerParameters(),fullRobotModel, atlasRobotModel, referenceFrames, yoTime, robotModel, registry);
   }

   public OffsetAndYawRobotInitialSetup  setStartingLocationOffset(StartingLocation startFromHere)
   {
      startingPos = getStartingLocationOffset(startFromHere);
      return startingPos;
   }

   private void checkAndPublishChestTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
      if (message.takeNextData().getWalkingStatus() != 0)
      {
         if(tmpcounter == 0)
         {callDoorTiminingBehavior();}
         //createAndPublishChestTrajectory(ReferenceFrame.getWorldFrame(),drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame()); //call your door opening behavior form this code point

         tmpcounter++;
      }
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

         behaviorDispatcher.closeAndDispose();
         ros2Node = null;
         yoTime = null;
         robot = null;
         fullRobotModel = null;
         //worldFrame = null;
         referenceFrames = null;
         robotDataReceiver = null;
         behaviorDispatcher = null;
         newStatusReference = null;
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

   private void addMapping(AvatarStepUp.StartingLocation startingLocation, OffsetAndYawRobotInitialSetup initialPosofRobot)
   {
      initalLocation.put(startingLocation.toString(),initialPosofRobot);
   }

   public OffsetAndYawRobotInitialSetup getStartingLocationOffset(StartingLocation tmp)
   {
      OffsetAndYawRobotInitialSetup startPosition = initalLocation.get(tmp.toString());
      return startPosition;
   }




   private BehaviorDispatcher<HumanoidBehaviorType> setupBehaviorDispatcher(String robotName, FullHumanoidRobotModel fullRobotModel, Ros2Node ros2Node,
                                                                            YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> {
                                              if(robotDataReceiver!=null && s!=null)
                                                 robotDataReceiver.receivedPacket(s.takeNextData());
                                           });

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      ROS2Tools.createCallbackSubscription(ros2Node, BehaviorControlModePacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName),
                                           s -> desiredBehaviorControlSubscriber.receivedPacket(s.takeNextData()));

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      ROS2Tools.createCallbackSubscription(ros2Node, HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName),
                                           s -> desiredBehaviorSubscriber.receivedPacket(s.takeNextData()));

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);
//      yoGraphicsListRegistry.registerYoGraphicsList(stepUpDoor.getSphereRobot().getYoGraphicsList());


      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(robotName, yoTime, robotDataReceiver, desiredBehaviorControlSubscriber,
                                                                              desiredBehaviorSubscriber, ros2Node, yoVariableServer, HumanoidBehaviorType.class,
                                                                              HumanoidBehaviorType.STOP, registry, yoGraphicsListRegistry);

      return ret;
   }

   private CapturePointUpdatable createCapturePointUpdateable(DRCSimulationTestHelper testHelper, YoVariableRegistry registry,
                                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      testHelper.createSubscriberFromController(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber::receivedPacket);

      CapturePointUpdatable ret = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);

      return ret; // so this is continuously getting updated
   }

   //start writing your step up code here now
   @Test
   public void stepUpSmall() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.3;
      double swingHeight = 0.15; //maybe needs to be changed
      //walkingStair(stepHeight, swingHeight);
      setPublishers(IS_LEFTARM_ON, IS_RIGHTARM_ON, IS_CHEST_ON, IS_FOOTSTEP_ON, IS_PELVIS_ON, IS_PAUSING_ON,  stepHeight, swingHeight);

   }


   /**
    * initialize all the publishers you want to send to the messages and create environemnt too
    * @param leftArm - set true for left arm trajectory
    * @param rightArm - set true for right arm trajectory
    * @param chest - set true for chest trajectory
    * @param isfootstepsON - set true to initialize preset footsteps till the steps and then the AStarfootstep planner kicks in
    * @param pelvis - set true for setting nomial pelvis height
    * @param pauseWhileWalking - set true to puase just before the hole and complete bending action
    * @param stepHeight - environment parameter
    * @param swingHeight - swing phase parameter
    * @throws SimulationExceededMaximumTimeException
    */

   public void setPublishers(boolean leftArm, boolean rightArm, boolean chest, boolean isfootstepsON, boolean pelvis, boolean pauseWhileWalking, double stepHeight, double swingHeight) throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = setTestEnvironment(stepHeight);

      FootstepDataListMessage footsteps = createFootSteps(robotModel, stepHeight, swingHeight);

      if(startWithoutBeingTriggeredFromMessagePacket == true)
      {
         if(tmpcounter == 0)
         {
            callDoorTiminingBehavior();
         }
         tmpcounter++;
      }

      if(leftArm)
      {
         ArmTrajectoryMessage leftArmTrajectoryMessages = createArmLeftTrajectory();
         drcSimulationTestHelper.publishToController(leftArmTrajectoryMessages);
      }

      if(rightArm)
      {
         ArmTrajectoryMessage rightArmTrajectoryMessages = createArmRightTrajectory();
         drcSimulationTestHelper.publishToController(rightArmTrajectoryMessages);
      }

      if(chest)
      {
         createAndPublishChestTrajectory(ReferenceFrame.getWorldFrame(),drcSimulationTestHelper.getReferenceFrames().getPelvisZUpFrame());
      }

      if(isfootstepsON)
      {

         drcSimulationTestHelper.publishToController(footsteps);

      }

      if(pelvis)
      {
         PelvisHeightTrajectoryMessage pelvisDHeight = createPelvisZUp(stepHeight);
         drcSimulationTestHelper.publishToController(pelvisDHeight);
      }

      if(pauseWhileWalking)
      {
         pauseWhileWalking(IS_PAUSING_ON);
      }

      // robot fell
//      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime +2.0*initialFinalTransfer + 12.0));
      // robot did not fall but did not reach goal
//      assertreached(footsteps);
      ThreadTools.sleepForever(); //does not kill the simulation
   }


   private void callDoorTiminingBehavior()
   {

      SearchAndKickBehavior searchAndKickBehavior = new SearchAndKickBehavior(getSimpleRobotName(), ros2Node, yoTime, referenceFrames, fullRobotModel, robotModel, yoDoubleSupport, stepUpDoor);
      HumanoidBehaviorTypePacket requestkickball = HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.SEARCH_AND_KICK_BEHAVIOR);
      System.out.println("behavior byte info :- ");

      drcSimulationTestHelper.createPublisher(HumanoidBehaviorTypePacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(drcSimulationTestHelper.getRobotName())).publish(requestkickball);
      behaviorDispatcher.addBehavior(HumanoidBehaviorType.SEARCH_AND_KICK_BEHAVIOR, searchAndKickBehavior);
      behaviorDispatcher.start();
   }

   /**
    * create right arm trajectory in joint space
    * @return Arm Trajectory Message
    */
   private ArmTrajectoryMessage createArmLeftTrajectory()
   {

      ArrayList<OneDoFJointTrajectoryMessage> leftandrightinfo = new ArrayList<>();

      ArmTrajectoryMessage leftHandMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT);


      ArmJointName[] armJointName = getArmJointNames();

      ArrayList<OneDoFJointTrajectoryMessage> leftArmTrajectory = new ArrayList<>();


      for(int armJointindex = 0 ; armJointindex < getArmDoF(); ++armJointindex)
      {
         OneDoFJointTrajectoryMessage leftJointTrajectory = new OneDoFJointTrajectoryMessage();


         for (int trajectoryPointIndex =0; trajectoryPointIndex < getArmTrajectoryPoints(); ++trajectoryPointIndex)
         {
            leftJointTrajectory.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage((double) (2*trajectoryPointIndex +1), getRandomJointAngle(RobotSide.LEFT,armJoint[armJointindex],fullRobotModel),(double) 0));

         }
         leftHandMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(leftJointTrajectory);

         leftArmTrajectory.add(leftJointTrajectory);

      }

      return leftHandMessage;
   }

   /**
    * create left arm trajectory in joint space
    * @return Arm Trajectory Message
    */
   private ArmTrajectoryMessage createArmRightTrajectory()
   {

      ArmTrajectoryMessage rightHandMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT);


      ArrayList<OneDoFJointTrajectoryMessage> rightArmTrajectory = new ArrayList<>();

      for(int armJointindex = 0 ; armJointindex < getArmDoF(); ++armJointindex)
      {

         OneDoFJointTrajectoryMessage rightJointTrajectory = new OneDoFJointTrajectoryMessage();

         for (int trajectoryPointIndex =0; trajectoryPointIndex < getArmTrajectoryPoints(); ++trajectoryPointIndex)
         {

            rightJointTrajectory.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage((double) (2*trajectoryPointIndex +1), getRandomJointAngle(RobotSide.RIGHT, armJoint[armJointindex], fullRobotModel), (double) 0));
         }

         rightHandMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(rightJointTrajectory);

         rightArmTrajectory.add(rightJointTrajectory);
      }

      return rightHandMessage;
   }

   /**
    * the first line is when the message of pausing kicks in. You need to adjust the timing manually. If the robot is in middle of completing an action it will complete it before
    * stopping. The second time is w.r.t to the first one (e.g 3s pause). After the motion is resume the Reference frames somehow get mixed up and the whole controller crashes.
    * Should be kept false for the meanwhile. Should update soon
    */
   private void pauseWhileWalking(boolean pauseONorOFF) throws SimulationExceededMaximumTimeException
   {

      if(pauseONorOFF)
      {
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.5);
         PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
         drcSimulationTestHelper.publishToController(pauseWalkingMessage);
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
         PauseWalkingMessage resumeWalkingMessag = HumanoidMessageTools.createPauseWalkingMessage(false);
         drcSimulationTestHelper.publishToController(resumeWalkingMessag);
      }
   }

   /**
    * method to create footsteps using initial pose and final pose with  AStar footstep planner. Called in createFootsteps method
    * @param stepLength
    * @param stepWidth
    * @return footstepdatamessage
    */
   private FootstepDataMessage[] createFootstepUsingFootstepPlanner(double stepLength, double stepWidth)
   {

      Point3D initialWaypoint = waypointtotransfer; //transfer the previous waypoint to a local variable

      Point2D initialStance = new Point2D(initialWaypoint.getX(), initialWaypoint.getY());
      FramePose2D initiaWaypoint2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialStance, 0.0);

      double finalXVale = (1+2.5*stepLength) +0.2;
      Point2D finalStance = new Point2D( finalXVale,stepWidth);
      FramePose2D finalWaypoint2d = new FramePose2D(ReferenceFrame.getWorldFrame(), finalStance, 0.0);

      RobotSide initialSide = RobotSide.LEFT;
      // starting foot position for left
      FramePose3D initialWaypoint3d = FlatGroundPlanningUtils.poseFormPose2d(initiaWaypoint2d, 0.0);
      FramePose3D finalWaypoint3d = FlatGroundPlanningUtils.poseFormPose2d(finalWaypoint2d,0.0);
      SimpleFootstep simpleFootstep = new SimpleFootstep(RobotSide.LEFT, finalWaypoint3d);

      FootstepPlan footstep = PlannerTools.runPlanner(planner, initialWaypoint3d, initialSide, finalWaypoint3d,null, true);



      numberOfFootstepByPlanner = footstep.getNumberOfSteps();
      FootstepDataMessage[] footsteps = new FootstepDataMessage[numberOfFootstepByPlanner];
      RobotSide side = RobotSide.LEFT;
      for (int footstepIndex = 0; footstepIndex < footstep.getNumberOfSteps(); footstepIndex++)
      {
         Point3D waypoint = new Point3D(footstep.getFootstep(footstepIndex).getSoleFramePose().getPosition().getX(), stepWidth, 0.0);
         FrameQuaternion orientation = new FrameQuaternion();
         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, waypoint, orientation);
         footsteps[footstepIndex] = footstepDataMessage;
         side = side.getOppositeSide();
         stepWidth = -stepWidth;
      }

      return footsteps;
   }

   /**
    * creates chest waypoints to be reached with the set time limit with publishes it after all the waypoints are set. THe ordering is an issue. he sequence is the very first message you publish
    * and then the add all the remaining stance in a queue with the same message_ID (in our case it is -1). THey get executed from last to the second form top. TrajectoryTime is w.r.t
    * to the simulation.
    * @param dataframe
    * @param trajectoryFrame
    */
   private void createAndPublishChestTrajectory(ReferenceFrame dataframe, ReferenceFrame trajectoryFrame)
   {
      double trajectoryTime = 10.5;
      FrameQuaternion chestOrientation1 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation2 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation3 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      FrameQuaternion chestOrientation4 = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      //chestOrientation.appendYawRotation(2.0); //there also these append methods that you can use to mention only yaw,roll or pitch angles.
      double leanAngle = 30.0; //original values 20.0 and yaw was -2.36
      chestOrientation1.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.00, Math.toRadians(leanAngle), 0.0);
      //Quaternion desiredchestOrientation = new Quaternion(chestOrientation);
      chestOrientation2.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), -2.36, 0.0, 0.0);
      chestOrientation3.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.2);
      chestOrientation4.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FrameQuaternion[] desiredChestOrientations = {chestOrientation1, chestOrientation2, chestOrientation3,chestOrientation4};

      //ChestTrajectoryMessage chestPoint = new ChestTrajectoryMessage();
      //executes this one first then the goes in reverse starting from the ver last one
      ChestTrajectoryMessage bend = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredChestOrientations[1], dataframe, trajectoryFrame);
      drcSimulationTestHelper.publishToController(bend);

      ChestTrajectoryMessage straight = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime + 5.5, desiredChestOrientations[3], dataframe, trajectoryFrame);
      straight.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      straight.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      //bend.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(yaw.getSequenceId());
      drcSimulationTestHelper.publishToController(straight);

   }

   /**
    * sets trajectory waypoints for pelvis. gets the Z values from when the simulation starts (time -  'simualateandbloack..' secs later) and sets the new desired
    * Z position w.r.t to the world reference frame to be reached within the given time limit.
    * @param stepHeight
    * @return
    */
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

   /**
    *  methods to create and set waypoints. Called in create footsteps method
    */
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

         }
         else if(i == 2) //left
         {
            footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
            //quaternion.setYawPitchRoll(-0.1, 0.0, 0.0);
            Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().add();
            waypoint1.set(xways[i-2] - 0.2, 0.13, zways[i-1]+0.1 );
            Point3D waypoint2 = footstepDataMessage.getCustomPositionWaypoints().add();
            waypoint2.set(xways[i-1] - 0.02, 0.13, zways[i-1] );

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

      }

      footstepDataMessage.setSwingDuration(swingTime);
      footstepDataMessage.setTransferDuration(transferTime);

      return footstepDataMessage;
   }

   /**
    * takes in appropriate paramters and creates the whole list of footstespdatamessage that is then passed to the controller using the publisher
    * @param robotModel
    * @param stepHeight
    * @param swingHeight
    * @return
    */
   private FootstepDataListMessage createFootSteps(DRCRobotModel robotModel,double stepHeight, double swingHeight)
   {
      //create a list of desired footstep
      double stepWidth = 0.13;
      double swingTime = 2.0;
      double defaultTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, 4);
      RobotSide tempright = RobotSide.RIGHT;
      MovingReferenceFrame soleZUpFramel = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(robotSides[0]); //left side
      MovingReferenceFrame soleZUpFramer = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(tempright); //right side
      FramePose3D solereferencel = new FramePose3D(soleZUpFramel);
      FramePose3D solereferencer = new FramePose3D(soleZUpFramer);
      solereferencel.changeFrame(ReferenceFrame.getWorldFrame());
      double xsole = solereferencel.getPosition().getX();
      double ysolel = solereferencel.getPosition().getY();
      double ysoler = solereferencer.getPosition().getY();
      double zsole = solereferencel.getPosition().getZ();

      boolean LOCAL_DEBUG = false;

      for (int i = 0 ; i < 4; i++)
      {
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
            footstepDataListMessage.getFootstepDataList().add().set(footsteps(i,robotSides[i] ,waypoint, frameQuaternion, swingTime, defaultTransferTime));

         }
         else //right
         {
            Point3D waypoint = new Point3D(xsole, -stepWidth, zsole);
            FrameQuaternion frameQuaternion = new FrameQuaternion();
            footstepDataListMessage.getFootstepDataList().add().set(footsteps(i,robotSides[i] ,waypoint, frameQuaternion, swingTime, defaultTransferTime));
         }
         if(LOCAL_DEBUG)
         {
            System.out.println("x position sole: " + xsole);
            System.out.println("yr position sole: " + ysoler);
            System.out.println("z position sole: " + zsole);
         }
      }

      FootstepDataMessage[] toBeAddedFootsteps = createFootstepUsingFootstepPlanner(stepLength,stepWidth);
      for(int footstepIndex = 0; footstepIndex < toBeAddedFootsteps.length; footstepIndex++)
      {

         if (footstepIndex == 1)
         {

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

   public void setIS_CHEST_ON(boolean IS_CHEST_ON)
   {
      this.IS_CHEST_ON = IS_CHEST_ON;
   }

   public void setIS_FOOTSTEP_ON(boolean IS_FOOTSTEP_ON)
   {
      this.IS_FOOTSTEP_ON = IS_FOOTSTEP_ON;
   }

   public void setIS_LEFTARM_ON(boolean IS_LEFTARM_ON)
   {
      this.IS_LEFTARM_ON = IS_LEFTARM_ON;
   }

   public void setIS_PAUSING_ON(boolean IS_PAUSING_ON)
   {
      this.IS_PAUSING_ON = IS_PAUSING_ON;
   }

   public void setIS_PELVIS_ON(boolean IS_PELVIS_ON)
   {
      this.IS_PELVIS_ON = IS_PELVIS_ON;
   }

   public void setIS_RIGHTARM_ON(boolean IS_RIGHTARM_ON)
   {
      this.IS_RIGHTARM_ON = IS_RIGHTARM_ON;
   }

   private DRCRobotModel setTestEnvironment(double stepHeight)  throws SimulationExceededMaximumTimeException
   {

      setUpCamera();
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);
      assertTrue(success);
//      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(50.0); //change this line to manually click on simulate button
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

}



