package us.ihmc.avatar.stepUpPlanner;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.StepUpPlannerCostWeights;
import controller_msgs.msg.dds.StepUpPlannerErrorMessage;
import controller_msgs.msg.dds.StepUpPlannerParametersMessage;
import controller_msgs.msg.dds.StepUpPlannerPhase;
import controller_msgs.msg.dds.StepUpPlannerPhaseParameters;
import controller_msgs.msg.dds.StepUpPlannerRequestMessage;
import controller_msgs.msg.dds.StepUpPlannerRespondMessage;
import controller_msgs.msg.dds.StepUpPlannerStepParameters;
import controller_msgs.msg.dds.StepUpPlannerVector2;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.SingleStepEnvironment;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.dataBuffer.DataEntry;

public abstract class AvatarStepUpPlannerTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
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

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   Ros2Publisher<StepUpPlannerParametersMessage> parametersPublisher;
   Ros2Publisher<StepUpPlannerRequestMessage> requestPublisher;
   StepUpPlannerRespondMessage receivedRespond;
   private int numberOfExceptions = 0;
   private static final int NUMBER_OF_EXCEPTIONS_TO_PRINT = 5;
   boolean parametersAcked = false;

   protected void walkUpToHighStep(double stepHeight) throws SimulationExceededMaximumTimeException
   {

      SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 1.0);
      OffsetAndYawRobotInitialSetup offset = new OffsetAndYawRobotInitialSetup(0.6, 0.0, 0.0, 0.0);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      drcSimulationTestHelper.setStartingLocation(offset);
      drcSimulationTestHelper.createSimulation("WalkingUpToHighPlatformtest");
      Point3D cameraFix = new Point3D(1.1281, 0.0142, 1.0528);
      Point3D cameraPosition = new Point3D(0.2936, -5.531, 1.7983);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      createTorqueGraph(drcSimulationTestHelper.getSimulationConstructionSet(), getRobotModel().createHumanoidFloatingRootJointRobot(false));

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stepUpPlanner_javaNode");

      ExceptionTools.handle(() -> parametersPublisher = ros2Node.createPublisher(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerParametersMessage.class),
                                                                                 "/us/ihmc/stepUpPlanner/parameters"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ExceptionTools.handle(() -> requestPublisher = ros2Node.createPublisher(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerRequestMessage.class),
                                                                              "/us/ihmc/stepUpPlanner/request"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ExceptionTools.handle(() -> ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerRespondMessage.class),
                                                              this::acceptRespondMessage,
                                                              "/us/ihmc/stepUpPlanner/respond"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ExceptionTools.handle(() -> ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerErrorMessage.class),
                                                              this::acceptErrorMessage,
                                                              "/us/ihmc/stepUpPlanner/errors"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);


      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      StepUpPlannerParametersMessage parameters = fillParametersMessage(getRobotModel().getWalkingControllerParameters().getSteppingParameters());
      LogTools.info("Sending parameters.");
      publishParameters(parameters);
      int loop = 0;
      
      while (!parametersAcked && loop < 20)
      {
         ThreadTools.sleep(500);
         LogTools.info("Waiting to receive parameters ack.");

         if (loop == 10)
         {
            LogTools.warn("Sending parameters again.");
            publishParameters(parameters);
         }

         ++loop;
      }
      assertTrue(loop != 10);

      StepUpPlannerRequestMessage request = fillRequestMessage(stepHeight);
      LogTools.info("Sending request.");
      publishRequest(request);
      loop = 0;
      
      while (receivedRespond == null && loop < 20)
      {
         ThreadTools.sleep(500);
         LogTools.info("Waiting to receive respond.");

         if (loop == 10)
         {
            LogTools.warn("Sending request again.");
            publishRequest(request);
         }

         ++loop;
      }
      assertTrue(loop != 20);


      for (int i = 0; i < receivedRespond.getFoostepMessages().size(); ++i)
      {
         drcSimulationTestHelper.publishToController(receivedRespond.getFoostepMessages().get(i));
      }

      LogTools.info("Publishing CoM message with ID:"
            + receivedRespond.getComMessages().get(0).getEuclideanTrajectory().getQueueingProperties().getMessageId());
      drcSimulationTestHelper.publishToController(receivedRespond.getComMessages().get(0));

      for (int i = receivedRespond.getComMessages().size() - 1; i > 0; --i)
      {
         LogTools.info("Publishing CoM message with ID:"
               + receivedRespond.getComMessages().get(i).getEuclideanTrajectory().getQueueingProperties().getMessageId());
         drcSimulationTestHelper.publishToController(receivedRespond.getComMessages().get(i));
      }

      //      for (int i = 0; i < receivedRespond.getComMessages().size(); ++i)
      //      {
      //         drcSimulationTestHelper.publishToController(receivedRespond.getComMessages().get(i));
      //         ThreadTools.sleep(500);
      //      }


      //      FootstepDataListMessage footsteps = createFootstepsForHighStepUp(environment.getStepsCenter());
      //      PelvisHeightTrajectoryMessage pelvisHeightTrajectory = createPelvisTrajectoryForHighStepUp(environment.getStepsCenter());

      //      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      //      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      //      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();
      //
      //      drcSimulationTestHelper.publishToController(footsteps);
      //            drcSimulationTestHelper.publishToController(pelvisHeightTrajectory);
      //
      //      int numberOfSteps = footsteps.getFootstepDataList().size();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(receivedRespond.getTotalDuration() * 1.2);

      printMinMax(drcSimulationTestHelper.getSimulationConstructionSet());

      assertReachedGoal(receivedRespond.getFoostepMessages().getLast());
   }

   private void publishParameters(StepUpPlannerParametersMessage parametersMessage)
   {
      try
      {
         parametersPublisher.publish(parametersMessage);
      }
      catch (Exception e)
      {
         if (numberOfExceptions <= NUMBER_OF_EXCEPTIONS_TO_PRINT)
         {
            e.printStackTrace();

            if (++numberOfExceptions > NUMBER_OF_EXCEPTIONS_TO_PRINT)
            {
               LogTools.error("Stopping to print exceptions after {}.", NUMBER_OF_EXCEPTIONS_TO_PRINT);
            }
         }
      }
   }

   private void publishRequest(StepUpPlannerRequestMessage requestMessage)
   {
      try
      {
         requestPublisher.publish(requestMessage);
      }
      catch (Exception e)
      {
         if (numberOfExceptions <= NUMBER_OF_EXCEPTIONS_TO_PRINT)
         {
            e.printStackTrace();

            if (++numberOfExceptions > NUMBER_OF_EXCEPTIONS_TO_PRINT)
            {
               LogTools.error("Stopping to print exceptions after {}.", NUMBER_OF_EXCEPTIONS_TO_PRINT);
            }
         }
      }
   }

   private void acceptRespondMessage(Subscriber<StepUpPlannerRespondMessage> subscriber)
   {
      StepUpPlannerRespondMessage incomingData = subscriber.takeNextData();
      if (incomingData != null)
      {
         receivedRespond = incomingData;
      }
      else
      {
         LogTools.warn("Received null from takeNextData()");
      }
   }

   private void acceptErrorMessage(Subscriber<StepUpPlannerErrorMessage> subscriber)
   {
      StepUpPlannerErrorMessage incomingData = subscriber.takeNextData();
      if (incomingData != null)
      {
         if (incomingData.getErrorCode() > 0)
         {
            LogTools.error("Recevied error: " + incomingData.getErrorDescriptionAsString());
            assertTrue(false);
         }
         else
         {
            LogTools.info("Received ack for message with ID: " + incomingData.getSequenceIdReceived());
            parametersAcked = true;
         }
      }
      else
      {
         LogTools.warn("Received null from takeNextData()");
      }
   }

   private StepUpPlannerParametersMessage fillParametersMessage(SteppingParameters steppingParameters)
   {
      StepUpPlannerParametersMessage msg = new StepUpPlannerParametersMessage();
      
      ArrayList<StepUpPlannerStepParameters> leftSteps = new ArrayList<StepUpPlannerStepParameters>();
      ArrayList<StepUpPlannerStepParameters> rightSteps = new ArrayList<StepUpPlannerStepParameters>();
      
      double rearOfFoot  = -steppingParameters.getFootLength() / 2.0;
      double frontOfFoot = steppingParameters.getFootLength() / 2.0;
      double toeWidth = steppingParameters.getToeWidth();
      double heelWidth = steppingParameters.getFootWidth();
      
      for (int i = 0; i < 5; ++i) {
         StepUpPlannerStepParameters newStep = new StepUpPlannerStepParameters();
         StepUpPlannerVector2 newVertex = new StepUpPlannerVector2();
         
         newVertex.setX(frontOfFoot);
         newVertex.setY(toeWidth/2.0);
         newStep.getFootVertices().add().set(newVertex);
         
         newVertex.setX(frontOfFoot);
         newVertex.setY(-toeWidth/2.0);
         newStep.getFootVertices().add().set(newVertex);
         
         newVertex.setX(rearOfFoot);
         newVertex.setY(-heelWidth/2.0);
         newStep.getFootVertices().add().set(newVertex);
         
         newVertex.setX(rearOfFoot);
         newVertex.setY(heelWidth/2.0);
         newStep.getFootVertices().add().set(newVertex);
         
         leftSteps.add(new StepUpPlannerStepParameters(newStep));
         rightSteps.add(new StepUpPlannerStepParameters(newStep));
      }

      leftSteps.get(0).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(0).setState(StepUpPlannerStepParameters.STAND);

      leftSteps.get(1).setState(StepUpPlannerStepParameters.SWING);
      rightSteps.get(1).setState(StepUpPlannerStepParameters.STAND);

      leftSteps.get(2).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(2).setState(StepUpPlannerStepParameters.STAND);

      leftSteps.get(3).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(3).setState(StepUpPlannerStepParameters.SWING);

      leftSteps.get(4).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(4).setState(StepUpPlannerStepParameters.STAND);

      for (int p = 0; p < 5; ++p) {
         StepUpPlannerPhaseParameters newSettings = new StepUpPlannerPhaseParameters();
         newSettings.getLeftStepParameters().set(leftSteps.get(p));
         newSettings.getRightStepParameters().set(rightSteps.get(p));
         msg.getPhasesParameters().add().set(newSettings);
      }
      
      msg.setPhaseLength(30);
      msg.setSolverVerbosity(1);
      msg.setMaxLegLength(1.5);
      //      msg.setIpoptLinearSolver("ma27");
      msg.setFinalStateAnticipation(0.3);
      msg.setStaticFrictionCoefficient(0.5);
      msg.setTorsionalFrictionCoefficient(0.1);

      double N = msg.getPhaseLength() * msg.getPhasesParameters().size();

      StepUpPlannerCostWeights weights = new StepUpPlannerCostWeights();
      
      weights.setCop(10.0/N);
      weights.setTorques(1.0/N);
      weights.setControlMultipliers(0.1/N);
      weights.setFinalControl(1.0);
      weights.setMaxControlMultiplier(0.1);
      weights.setFinalState(10.0);
      weights.setControlVariations(1.0/N);
      weights.setDurationsDifference(5.0 / msg.getPhasesParameters().size());
      

      msg.getCostWeights().set(weights);

      msg.setSequenceId(1);
      
      msg.setSendComMessages(false);
      msg.setMaxComMessageLength(40);
      msg.setIncludeComMessages(true);
      
      msg.setSendFootstepMessages(false);
      msg.setIncludeFootstepMessages(true);

      return msg;
   }

   private StepUpPlannerRequestMessage fillRequestMessage(double stepHeight)
   {
      StepUpPlannerRequestMessage msg = new StepUpPlannerRequestMessage();
      
      ReferenceFrame initialCoMFrame = drcSimulationTestHelper.getReferenceFrames().getCenterOfMassFrame();
      FramePose3D comPose = new FramePose3D(initialCoMFrame);
      comPose.changeFrame(ReferenceFrame.getWorldFrame());

      msg.getInitialComPosition().set(comPose.getX(), comPose.getY(), comPose.getZ());
      msg.getInitialComVelocity().setToZero();
      msg.getDesiredComPosition().set(comPose.getX() + 0.6, comPose.getY(), comPose.getZ() + stepHeight);
      msg.getDesiredComVelocity().setToZero();
      
      FrameQuaternion identityQuaternion = new FrameQuaternion();
      identityQuaternion.set(0, 0, 0, 1.0);

      msg.getPhases().clear();

      MovingReferenceFrame initialLeft = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(RobotSide.LEFT);
      FramePose3D initialLeftPose = new FramePose3D(initialLeft);
      initialLeftPose.changeFrame(ReferenceFrame.getWorldFrame());

      MovingReferenceFrame initialRight = drcSimulationTestHelper.getReferenceFrames().getSoleZUpFrame(RobotSide.RIGHT);
      FramePose3D initialRightPose = new FramePose3D(initialRight);
      initialRightPose.changeFrame(ReferenceFrame.getWorldFrame());

      Pose3D l1 = new Pose3D();
      Pose3D l2 = new Pose3D();
      Pose3D r1 = new Pose3D();
      Pose3D r2 = new Pose3D();

      l1.set(initialLeftPose);
      r1.set(initialRightPose);

      StepUpPlannerPhase newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(l1);
      newPhase.getRightFootPose().set(r1);
      newPhase.setMinimumDuration(0.5);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(0.8);

      newPhase = msg.getPhases().add();
      newPhase.getRightFootPose().set(r1);
      newPhase.setMinimumDuration(0.5);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.2);

      l2.set(l1);
      l2.getPosition().setX(l1.getPosition().getX() + 0.6);
      l2.getPosition().setZ(l1.getPosition().getZ() + stepHeight);

      newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(l2);
      newPhase.getRightFootPose().set(r1);
      newPhase.setMinimumDuration(0.5);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(0.8);

      newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(l2);
      newPhase.setMinimumDuration(0.5);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.2);

      r2.set(r1);
      r2.getPosition().setX(r1.getPosition().getX() + 0.6);
      r2.getPosition().setZ(r1.getPosition().getZ() + stepHeight);

      newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(l2);
      newPhase.getRightFootPose().set(r2);
      newPhase.setMinimumDuration(0.5);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(0.8);

      msg.setDesiredLegLength(comPose.getPositionDistance(initialLeftPose.getPosition()));

      msg.getLeftDesiredFinalControl().getCop().setX(0.0);
      msg.getLeftDesiredFinalControl().getCop().setY(0.0);
      msg.getRightDesiredFinalControl().set(msg.getLeftDesiredFinalControl());


      double desiredLeftMultiplier = 9.81/(2.0*(msg.getDesiredComPosition().getZ() -
            l2.getPosition().getZ()));

      msg.getLeftDesiredFinalControl().setMultiplier(desiredLeftMultiplier);

      double desiredRightMultiplier = 9.81 / (2.0 * (msg.getDesiredComPosition().getZ() - r2.getPosition().getZ()));

      msg.getRightDesiredFinalControl().setMultiplier(desiredRightMultiplier);
      
      return msg;
   }


   private void assertReachedGoal(FootstepDataListMessage footsteps)
   {
      int numberOfSteps = footsteps.getFootstepDataList().size();
      Point3D lastStep = footsteps.getFootstepDataList().get(numberOfSteps - 1).getLocation();
      Point3D nextToLastStep = footsteps.getFootstepDataList().get(numberOfSteps - 2).getLocation();

      Point3D midStance = new Point3D();
      midStance.interpolate(lastStep, nextToLastStep, 0.5);

      Point3D midpoint = new Point3D(midStance);
      midpoint.addZ(1.0);

      Point3D bounds = new Point3D(0.25, 0.25, 0.3);

      BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(midpoint, bounds);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox3D);
   }

   private void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
      }
   }

   private void getPinJoints(Robot robot, List<PinJoint> pinJoints)
   {
      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoints);
      }
   }

   private void createTorqueGraph(SimulationConstructionSet scs, Robot robot)
   {
      StandardSimulationGUI window = scs.getGUI();
      GraphArrayPanel panel = window.getGraphArrayPanel();
      ArrayList<YoGraph> graphs = panel.getGraphsOnThisPanel();

      if (graphs.size() != 0)
         return;

      List<PinJoint> pinJoints = new ArrayList<PinJoint>();

      getPinJoints(robot, pinJoints);

      List<String> torsoJoints = new ArrayList<String>();
      List<String> leftLegJoints = new ArrayList<String>();
      List<String> rightLegJoints = new ArrayList<String>();

      for (PinJoint joint : pinJoints)
      {
         String name = joint.getTauYoVariable().getName();

         if (name.contains("l_leg"))
         {
            leftLegJoints.add(name);
         }
         else if (name.contains("r_leg"))
         {
            rightLegJoints.add(name);
         }
         else if (name.contains("back"))
         {
            torsoJoints.add(name);
         }
      }

      addGraph(scs, torsoJoints);
      addGraph(scs, leftLegJoints);
      addGraph(scs, rightLegJoints);
   }

   private void addGraph(SimulationConstructionSet scs, List<String> torsoJoints)
   {
      scs.setupGraph(torsoJoints.toArray(new String[0]));
   }

   private void printMinMax(SimulationConstructionSet scs)
   {
      System.out.println("-------------------------------------------------------------");
      StandardSimulationGUI window = scs.getGUI();
      GraphArrayPanel panel = window.getGraphArrayPanel();
      ArrayList<YoGraph> graphs = panel.getGraphsOnThisPanel();

      for (YoGraph graph : graphs)
      {
         ArrayList<DataEntry> entries = graph.getEntriesOnThisGraph();
         entries.forEach(entry -> System.out.println(entry.getVariableName() + " Max Torque: [" + Math.max(Math.abs(entry.getMin()), Math.abs(entry.getMax()))
               + "]"));
      }

      System.out.println("-------------------------------------------------------------");
   }

}