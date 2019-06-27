package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import com.jme3.math.Transform;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class PlannerToolboxDataSetTest
{
   // Whether to start the UI or not.
   public static boolean VISUALIZE = true;
   // For enabling helpful prints.
   private static boolean DEBUG = false;
   private static boolean VERBOSE = true;

   private static final List<DataSetName> dataSetsToTest = new ArrayList<>();
   static
   {
      dataSetsToTest.add(DataSetName._20190626_Plank);
      dataSetsToTest.add(DataSetName._20171215_214730_CinderBlockField);
      dataSetsToTest.add(DataSetName._20171026_131304_PlanarRegion_Ramp_2Story_UnitTest);
      dataSetsToTest.add(DataSetName._20171215_214801_StairsUpDown);
   }

   private static final double numberOfIterationsToAverage = 10;


   private FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   private AtomicReference<FootstepDataListMessage> uiFootstepPlanReference;
   private AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   private final AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   private final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);
   private final AtomicReference<Double> planningDuration = new AtomicReference<>(null);

   private static final String robotName = "testBot";
   private MultiStageFootstepPlanningModule toolboxModule;

   private RealtimeRos2Node ros2Node;

   private RemoteUIMessageConverter messageConverter = null;

   public final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      messageConverter = RemoteUIMessageConverter.createConverter(messager, robotName, pubSubImplementation);

      tryToStartModule(() -> setupFootstepPlanningToolboxModule());

      messager.registerTopicListener(FootstepPlanResponseTopic, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> uiReceivedResult.set(true));
      messager.registerTopicListener(PlannerTimeTakenTopic, planningDuration::set);

      uiFootstepPlanReference = messager.createInput(FootstepPlanResponseTopic);
      uiPlanningResultReference = messager.createInput(PlanningResultTopic);

      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      ros2Node.spin();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      if (VISUALIZE)
      {
         createUI(messager);
      }

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);
   }

   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   private FootstepPlannerParameters getPlannerParameters()
   {
      return new TestFootstepPlannerParameters();
   }

   private RobotContactPointParameters<RobotSide> getContactParameters()
   {
      return new TestContactPointParameters();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      ros2Node.destroy();

      messager.closeMessager();
      messageConverter.destroy();
      toolboxModule.destroy();

      if (ui != null)
         ui.stop();
      ui = null;

      uiFootstepPlanReference = null;
      uiPlanningResultReference = null;

      ros2Node = null;
      toolboxModule = null;

      messageConverter = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      plannerPlanReference.set(null);
      plannerResultReference.set(null);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      if (uiFootstepPlanReference != null)
         uiFootstepPlanReference.set(null);
      if (uiPlanningResultReference != null)
         uiPlanningResultReference.set(null);
      uiReceivedPlan.set(false);
      uiReceivedResult.set(false);

      expectedPlan.set(null);
      actualPlan.set(null);
      expectedResult.set(null);
      actualResult.set(null);
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         LogTools.error("Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   private void setupFootstepPlanningToolboxModule()
   {
      toolboxModule = new MultiStageFootstepPlanningModule(getRobotModel(), null, true, pubSubImplementation);
   }

   private DRCRobotModel getRobotModel()
   {
      return new TestRobotModel();
   }

   public void runAssertionsOnDataSet(DataSetName dataSetName)
   {
      List<DataSetName> dataSetNames = new ArrayList<>();
      dataSetNames.add(dataSetName);
      runAssertionsOnDataSets(dataSetNames);
   }

   public void runAssertionsOnDataSets(List<DataSetName> dataSetNames)
   {
      List<DataSet> dataSets = new ArrayList<>();
      for (DataSetName dataSetName : dataSetNames)
      {
         dataSets.add(DataSetIOTools.loadDataSet(dataSetName));
      }

      runAssertionsOnAllDataSets(dataSets);
   }

   public void runAssertionsOnAllDataSets(List<DataSet> allDatasets)
   {
      if (DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assertions.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      int numberOfTestedSets = 0;
      List<String> failingDatasets = new ArrayList<>();
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numberOfTestedSets++;

         double totalDuration = 0.0;
         double numberOfTests = 0.0;
         for (int trialNumber = 0; trialNumber < numberOfIterationsToAverage; trialNumber++)
         {
            resetAllAtomics();
            String errorMessagesForCurrentFile = runAssertions(dataset);
            if (!errorMessagesForCurrentFile.isEmpty())
            {
               numberOfFailingTests++;
               failingDatasets.add(dataset.getName());
            }

            if (DEBUG)
            {
               String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
               LogTools.info(dataset.getName() + " " + result);
            }

            if (trialNumber > numberOfIterationsToAverage / 2.0)
            {
               totalDuration += planningDuration.getAndSet(null);
               numberOfTests += 1.0;
            }
         }

         if (VERBOSE)
         {
            LogTools.info("Average duration = " + totalDuration / numberOfTests);
         }

         ThreadTools.sleep(50); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numberOfTestedSets;
      message += "\n Datasets failing: ";
      for (int i = 0; i < failingDatasets.size(); i++)
      {
         message += "\n" + failingDatasets.get(i);
      }
      if (VISUALIZE)
      {
         LogTools.info(message);
         ThreadTools.sleepForever();
      }
      else
      {
         Assertions.assertEquals(numberOfFailingTests, 0, message);
      }
   }

   public String runAssertions(DataSet dataset)
   {
      resetAllAtomics();
//      ThreadTools.sleep(1000);

      packPlanningRequest(dataset, messager);

      resetAllAtomics();
//      ThreadTools.sleep(1000);

      return findPlanAndAssertGoodResult(dataset);
   }

   private void packPlanningRequest(DataSet dataset, Messager messager)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, dataset.getPlannerInput().getStartPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, dataset.getPlannerInput().getGoalPosition());
      messager.submitMessage(PlannerTypeTopic, getPlannerType());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());

      double timeout = Double.parseDouble(dataset.getPlannerInput().getAdditionalData(getTimeoutFlag()).get(0));
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeoutTopic, timeout);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLengthTopic, Double.MAX_VALUE);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, getPlannerParameters());

      if (dataset.getPlannerInput().hasStartOrientation())
         messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, new Quaternion(dataset.getPlannerInput().getStartYaw(), 0.0, 0.0));
      if (dataset.getPlannerInput().hasGoalOrientation())
         messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, new Quaternion(dataset.getPlannerInput().getGoalYaw(), 0.0, 0.0));

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePathTopic, true);

      if (DEBUG)
         LogTools.info("Sending out planning request packet.");
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         LogTools.info("Processed an output from a remote planner.");

      plannerResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));
      plannerReceivedPlan.set(true);
      plannerReceivedResult.set(true);
   }

   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         SimpleFootstep footstep = footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);

         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (int i = 0; i < footstepMessage.getPredictedContactPoints2d().size(); i++)
            foothold.addVertex(footstepMessage.getPredictedContactPoints2d().get(i));
         foothold.update();
         footstep.setFoothold(foothold);
      }

      return footstepPlan;
   }

   private String getTimeoutFlag()
   {
      return getPlannerType().toString().toLowerCase() + "_timeout";
   }

   private String assertPlansAreValid(String datasetName, FootstepPlanningResult expectedResult, FootstepPlanningResult actualResult, FootstepPlan expectedPlan,
                                      FootstepPlan actualPlan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning results for " + datasetName + " are not equal: " + expectedResult + " and " + actualResult + ".\n",
                                 expectedResult.equals(actualResult));

      errorMessage += assertPlanIsValid(datasetName, expectedResult, expectedPlan, goal);
      errorMessage += assertPlanIsValid(datasetName, actualResult, actualPlan, goal);

      if (actualResult.validForExecution())
      {
         errorMessage += areFootstepPlansEqual(actualPlan, expectedPlan);
      }

      return errorMessage;
   }

   private String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plan, Point3D goal)
   {
      String errorMessage = "";

      if (!result.validForExecution())
      {
         errorMessage = "Planning result for " + datasetName + " is invalid, result was " + result;
      }
      else if (!PlannerTools.isGoalNextToLastStep(goal, plan))
      {
         errorMessage = datasetName + " did not reach goal. Made it to " + PlannerTools.getEndPosition(plan) + ", trying to get to " + goal;
      }

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            LogTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   private String areFootstepPlansEqual(FootstepPlan actualPlan, FootstepPlan expectedPlan)
   {
      String errorMessage = "";

      if (actualPlan.getNumberOfSteps() != expectedPlan.getNumberOfSteps())
      {
         errorMessage += "Actual Plan has " + actualPlan.getNumberOfSteps() + ", while Expected Plan has " + expectedPlan.getNumberOfSteps() + ".\n";
      }

      for (int i = 0; i < Math.min(actualPlan.getNumberOfSteps(), expectedPlan.getNumberOfSteps()); i++)
      {
         errorMessage += areFootstepsEqual(i, actualPlan.getFootstep(i), expectedPlan.getFootstep(i));
      }

      return errorMessage;
   }

   private String areFootstepsEqual(int footstepNumber, SimpleFootstep actual, SimpleFootstep expected)
   {
      String errorMessage = "";

      if (!actual.getRobotSide().equals(expected.getRobotSide()))
      {
         errorMessage += "Footsteps " + footstepNumber + " are different robot sides: " + actual.getRobotSide() + " and " + expected.getRobotSide() + ".\n";
      }

      FramePose3D poseA = new FramePose3D();
      FramePose3D poseB = new FramePose3D();

      actual.getSoleFramePose(poseA);
      expected.getSoleFramePose(poseB);

      if (!poseA.epsilonEquals(poseB, 1e-5))
      {
         errorMessage += "Footsteps " + footstepNumber + " have different poses: \n \t" + poseA.toString() + "\n and \n\t " + poseB.toString() + ".\n";
      }

      if (!actual.epsilonEquals(expected, 1e-5))

      {
         errorMessage += "Footsteps " + footstepNumber + " are not equal: \n \t" + actual.toString() + "\n and \n\t " + expected.toString() + ".\n";
      }

      return errorMessage;
   }

   private void createUI(Messager messager)
   {
      ApplicationRunner.runApplication(new Application()
      {
         @Override
         public void start(Stage stage) throws Exception
         {
            ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
            ui.show();
         }

         @Override
         public void stop() throws Exception
         {
            ui.stop();
            Platform.exit();
         }
      });

      double maxWaitTime = 5.0;
      double totalTime = 0.0;
      long sleepDuration = 100;

      while (ui == null)
      {
         if (totalTime > maxWaitTime)
            throw new RuntimeException("Timed out waiting for the UI to start.");
         ThreadTools.sleep(sleepDuration);
         totalTime += Conversions.millisecondsToSeconds(sleepDuration);
      }
   }

   private double totalTimeTaken;

   private String waitForResult(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";
      long waitTime = 10;
      while (conditionChecker.checkCondition())
      {
         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting for a result.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   private String validateResult(ConditionChecker conditionChecker, FootstepPlanningResult result, String prefix)
   {
      String errorMessage = "";

      if (!conditionChecker.checkCondition())
      {
         errorMessage += prefix + " failed to find a valid result. Result : " + result + "\n";
      }

      return errorMessage;
   }

   private String waitForPlan(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";

      while (conditionChecker.checkCondition())
      {
         long waitTime = 10;

         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting on plan.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   private void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
      {
         if (DEBUG)
            LogTools.info("Received a plan from the UI.");
         actualPlan.set(FootstepDataMessageConverter.convertToFootstepPlan(uiFootstepPlanReference.getAndSet(null)));
         uiReceivedPlan.set(false);
      }

      if (uiReceivedResult.get() && uiPlanningResultReference.get() != null)
      {
         if (DEBUG)
            LogTools.info("Received a result " + uiPlanningResultReference.get() + " from the UI.");
         actualResult.set(uiPlanningResultReference.getAndSet(null));
         uiReceivedResult.set(false);
      }
   }

   private void queryPlannerResults()
   {
      if (plannerReceivedPlan.get() && plannerPlanReference.get() != null && expectedPlan.get() == null)
      {
         if (DEBUG)
            LogTools.info("Received a plan from the planner.");
         expectedPlan.set(plannerPlanReference.getAndSet(null));
         plannerReceivedPlan.set(false);
      }

      if (plannerReceivedResult.get() && plannerResultReference.get() != null)
      {
         if (DEBUG)
            LogTools.info("Received a result " + plannerResultReference.get() + " from the planner.");
         expectedResult.set(plannerResultReference.getAndSet(null));
         plannerReceivedResult.set(false);
      }
   }

   public String findPlanAndAssertGoodResult(DataSet dataset)
   {
      totalTimeTaken = 0;
      double timeout = Double.parseDouble(dataset.getPlannerInput().getAdditionalData(getTimeoutFlag()).get(0));
      double maxTimeToWait = 2.0 * timeout;
      String datasetName = "";

      queryUIResults();
      queryPlannerResults();

      String errorMessage = "";

      if (DEBUG)
         LogTools.info("Waiting for result.");

      errorMessage += waitForResult(() -> actualResult.get() == null || expectedResult.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         LogTools.info("Received a result (actual = " + actualResult.get() + " expected = " + expectedResult.get() + ", checking it's validity.");

      errorMessage += validateResult(() -> actualResult.get().validForExecution() && expectedResult.get().validForExecution(), actualResult.get(), datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         LogTools.info("Results are valid, waiting for plan.");

      errorMessage += waitForPlan(() -> expectedPlan.get() == null || actualPlan.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         LogTools.info("Received a plan, checking it's validity.");

      FootstepPlanningResult expectedResult = this.expectedResult.getAndSet(null);
      FootstepPlanningResult actualResult = this.actualResult.getAndSet(null);
      FootstepPlan expectedPlan = this.expectedPlan.getAndSet(null);
      FootstepPlan actualPlan = this.actualPlan.getAndSet(null);

      uiReceivedResult.set(false);
      uiReceivedPlan.set(false);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      errorMessage += assertPlansAreValid(datasetName, expectedResult, actualResult, expectedPlan, actualPlan, dataset.getPlannerInput().getGoalPosition());

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      return errorMessage;
   }

   private static interface ConditionChecker
   {
      boolean checkCondition();
   }

   private class TestRobotModel implements DRCRobotModel
   {
      @Override
      public DRCRobotJointMap getJointMap()
      {
         return null;
      }

      @Override
      public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
      {
         return null;
      }

      @Override
      public HandModel getHandModel()
      {
         return null;
      }

      @Override
      public Transform getJmeTransformWristToHand(RobotSide side)
      {
         return null;
      }

      @Override
      public double getSimulateDT()
      {
         return 0;
      }

      @Override
      public double getEstimatorDT()
      {
         return 0;
      }

      @Override
      public double getStandPrepAngle(String jointName)
      {
         return 0;
      }

      @Override
      public DRCSensorSuiteManager getSensorSuiteManager()
      {
         return null;
      }

      @Override
      public LogSettings getLogSettings()
      {
         return null;
      }

      @Override
      public LogModelProvider getLogModelProvider()
      {
         return null;
      }

      @Override
      public String getSimpleRobotName()
      {
         return robotName;
      }

      @Override
      public CollisionBoxProvider getCollisionBoxProvider()
      {
         return null;
      }

      @Override
      public HighLevelControllerParameters getHighLevelControllerParameters()
      {
         return null;
      }

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
      {
         return null;
      }

      @Override
      public RobotDescription getRobotDescription()
      {
         return null;
      }

      @Override
      public FullHumanoidRobotModel createFullRobotModel()
      {
         return new TestFullRobotModel();
      }

      @Override
      public double getControllerDT()
      {
         return 0;
      }

      @Override
      public StateEstimatorParameters getStateEstimatorParameters()
      {
         return null;
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return null;
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return null;
      }

      @Override
      public RobotContactPointParameters<RobotSide> getContactPointParameters()
      {
         return getContactParameters();
      }

      @Override
      public HumanoidRobotSensorInformation getSensorInformation()
      {
         return null;
      }

      @Override
      public InputStream getWholeBodyControllerParametersFile()
      {
         return null;
      }

      @Override
      public FootstepPlannerParameters getFootstepPlannerParameters()
      {
         return getPlannerParameters();
      }

      @Override
      public VisibilityGraphsParameters getVisibilityGraphsParameters()
      {
         return new DefaultVisibilityGraphParameters();
      }
   }

   public class TestFullRobotModel implements FullHumanoidRobotModel
   {

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {

      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return null;
      }

      @Override
      public FloatingJointBasics getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBody getElevator()
      {
         return null;
      }

      @Override
      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public RigidBody getRootBody()
      {
         return null;
      }

      @Override
      public RigidBody getHead()
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public OneDoFJoint[] getOneDoFJoints()
      {
         return new OneDoFJoint[0];
      }

      @Override
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
      {

      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return new OneDoFJoint[0];
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {

      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {

      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return new IMUDefinition[0];
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return new ForceSensorDefinition[0];
      }

      @Override
      public double getTotalMass()
      {
         return 0;
      }

      @Override
      public RigidBody getChest()
      {
         return null;
      }

      @Override
      public RigidBody getPelvis()
      {
         return null;
      }

      @Override
      public OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName)
      {
         return null;
      }

      @Override
      public RigidBody getHand(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getHandControlFrame(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public void setJointAngles(RobotSide side, LimbName limb, double[] q)
      {

      }

      @Override
      public MovingReferenceFrame getFrameAfterLegJoint(RobotSide robotSegment, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public OneDoFJoint getLegJoint(RobotSide robotSegment, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public RigidBody getFoot(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(RobotSide robotSegment, LimbName limbName)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getEndEffectorFrame(RobotSide robotSegment, LimbName limbName)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getSoleFrame(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public SideDependentList<MovingReferenceFrame> getSoleFrames()
      {
         return null;
      }
   }

   public class TestFootstepPlannerParameters implements FootstepPlannerParameters
   {
      private boolean wiggleIntoConvexHull = true;
      private boolean rejectIfCannotFullyWiggleInside = false;
      private boolean returnBestEffortPlan = false;
      private boolean checkForBodyBoxCollisions = false;
      private boolean performHeuristicSearchPolicies = true;
      private int minimumStepsForBestEffortPlan = 3;
      private double cliffClearance = 0.1;
      private double cliffHeight = 0.05;
      private double maxStepLength = 0.5;
      private double maxStepWidth = 0.4;
      private double maxStepYaw = 0.5;
      private double maxStepZ = 0.25;
      private double maxXYWiggle = 0.1;
      private double maxYawWiggle = 0.09;
      private double minFootholdPercent = 0.7;
      private double minStepLength = -0.6;
      private double minStepWidth = 0.05;
      private double minStepYaw = -0.5;
      private double minSurfaceIncline = 0.7853981633974483;
      private double minXClearance = 0.22;
      private double minYClearance = 0.22;
      private double wiggleInsideDelta = 0.02;
      private double stepUpHeight = 1.5;
      private double stepDownHeight = 1.0;
      private double maxStepUpX = 0.5;
      private double maxStepDownX = 1.5;
      private double maxZPenetrationOnValleyRegions = Double.POSITIVE_INFINITY;
      private double idealFootstepWidth = 0.22;
      private double idealFootstepLength = 0.3;
      private double bodyGroundClearance = 0.25;
      private double bodyBoxWidth = 0.7;
      private double bodyBoxHeight = 1.5;
      private double bodyBoxDepth = 0.3;
      private double bodyBoxBaseX = 0.0;
      private double bodyBoxBaseY = 0.0;
      private double bodyBoxBaseZ = 0.25;
      private double finalTurnProximity = 1.0;

      @Override
      public boolean checkForBodyBoxCollisions()
      {
         return checkForBodyBoxCollisions;
      }

      @Override
      public boolean performHeuristicSearchPolicies()
      {
         return performHeuristicSearchPolicies;
      }

      @Override
      public double getIdealFootstepWidth()
      {
         return idealFootstepWidth;
      }

      @Override
      public double getIdealFootstepLength()
      {
         return idealFootstepLength;
      }

      @Override
      public double getWiggleInsideDelta()
      {
         return wiggleInsideDelta;
      }

      @Override
      public double getMaximumStepReach()
      {
         return maxStepLength;
      }

      @Override
      public double getMaximumStepYaw()
      {
         return maxStepYaw;
      }

      @Override
      public double getMinimumStepWidth()
      {
         return minStepWidth;
      }

      @Override
      public double getMinimumStepLength()
      {
         return minStepLength;
      }

      @Override
      public double getMinimumStepYaw()
      {
         return minStepYaw;
      }

      @Override
      public double getMaximumStepReachWhenSteppingUp()
      {
         return maxStepUpX;
      }

      @Override
      public double getMaximumStepZWhenSteppingUp()
      {
         return stepUpHeight;
      }

      @Override
      public double getMaximumStepXWhenForwardAndDown()
      {
         return maxStepDownX;
      }

      @Override
      public double getMaximumStepZWhenForwardAndDown()
      {
         return stepDownHeight;
      }

      @Override
      public double getMaximumStepZ()
      {
         return maxStepZ;
      }

      @Override
      public double getMinimumFootholdPercent()
      {
         return minFootholdPercent;
      }

      @Override
      public double getMinimumSurfaceInclineRadians()
      {
         return minSurfaceIncline;
      }

      @Override
      public boolean getWiggleIntoConvexHullOfPlanarRegions()
      {
         return wiggleIntoConvexHull;
      }

      @Override
      public boolean getRejectIfCannotFullyWiggleInside()
      {
         return rejectIfCannotFullyWiggleInside;
      }

      @Override
      public double getMaximumXYWiggleDistance()
      {
         return maxXYWiggle;
      }

      @Override
      public double getMaximumYawWiggle()
      {
         return maxYawWiggle;
      }

      @Override
      public double getMaximumZPenetrationOnValleyRegions()
      {
         return maxZPenetrationOnValleyRegions;
      }

      @Override
      public double getMaximumStepWidth()
      {
         return maxStepWidth;
      }

      @Override
      public double getCliffHeightToAvoid()
      {
         return cliffHeight;
      }

      @Override
      public double getMinimumDistanceFromCliffBottoms()
      {
         return cliffClearance;
      }

      @Override
      public boolean getReturnBestEffortPlan()
      {
         return returnBestEffortPlan;
      }

      @Override
      public int getMinimumStepsForBestEffortPlan()
      {
         return minimumStepsForBestEffortPlan;
      }

      @Override
      public double getBodyGroundClearance()
      {
         return bodyGroundClearance;
      }

      @Override
      public double getBodyBoxHeight()
      {
         return bodyBoxHeight;
      }

      @Override
      public double getBodyBoxDepth()
      {
         return bodyBoxDepth;
      }

      @Override
      public double getBodyBoxWidth()
      {
         return bodyBoxWidth;
      }

      @Override
      public double getBodyBoxBaseX()
      {
         return bodyBoxBaseX;
      }

      @Override
      public double getBodyBoxBaseY()
      {
         return bodyBoxBaseY;
      }

      @Override
      public double getBodyBoxBaseZ()
      {
         return bodyBoxBaseZ;
      }

      @Override
      public double getMinXClearanceFromStance()
      {
         return minXClearance;
      }

      @Override
      public double getMinYClearanceFromStance()
      {
         return minYClearance;
      }

      @Override
      public double getFinalTurnProximity()
      {
         return finalTurnProximity;
      }
   }

   private class TestContactPointParameters extends RobotContactPointParameters<RobotSide>
   {
      private static final double footWidthForControl = 0.13; //0.12; // 0.08;   //0.124887;
      private static final double toeWidthForControl = 0.085; //0.095; // 0.07;   //0.05;   //
      private static final double footLengthForControl = 0.22;
      private static final double footBackForControl = 0.085;
      private static final double ankleHeight = 0.084;

      public TestContactPointParameters()
      {
         super(new TestJointMap(), toeWidthForControl, footWidthForControl, footLengthForControl, new SideDependentList<>(TransformTools
               .createTranslationTransform(footLengthForControl / 2.0 - footBackForControl, 0.0, -ankleHeight), TransformTools.createTranslationTransform(footLengthForControl / 2.0 - footBackForControl, 0.0, -ankleHeight)));

         createDefaultFootContactPoints();
      }
   }

   private class TestJointMap implements LeggedJointNameMap<RobotSide>
   {

      @Override
      public ImmutablePair<RobotSide, LegJointName> getLegJointName(String jointName)
      {
         return null;
      }

      @Override
      public ImmutablePair<RobotSide, LimbName> getLimbName(String limbName)
      {
         return null;
      }

      @Override
      public String getJointBeforeFootName(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public RigidBodyTransform getSoleToParentFrameTransform(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public String getModelName()
      {
         return null;
      }

      @Override
      public JointRole getJointRole(String jointName)
      {
         return null;
      }

      @Override
      public NeckJointName getNeckJointName(String jointName)
      {
         return null;
      }

      @Override
      public SpineJointName getSpineJointName(String jointName)
      {
         return null;
      }

      @Override
      public String getRootBodyName()
      {
         return null;
      }

      @Override
      public String getUnsanitizedRootJointInSdf()
      {
         return null;
      }

      @Override
      public String getHeadName()
      {
         return null;
      }

      @Override
      public boolean isTorqueVelocityLimitsEnabled()
      {
         return false;
      }

      @Override
      public Set<String> getLastSimulatedJoints()
      {
         return null;
      }

      @Override
      public String[] getJointNamesBeforeFeet()
      {
         return new String[0];
      }

      @Override
      public RobotSide[] getRobotSegments()
      {
         return RobotSide.values;
      }

      @Override
      public RobotSide getEndEffectorsRobotSegment(String jointNameBeforeEndEffector)
      {
         return null;
      }

      @Override
      public LegJointName[] getLegJointNames()
      {
         return new LegJointName[0];
      }

      @Override
      public ArmJointName[] getArmJointNames()
      {
         return new ArmJointName[0];
      }

      @Override
      public SpineJointName[] getSpineJointNames()
      {
         return new SpineJointName[0];
      }

      @Override
      public NeckJointName[] getNeckJointNames()
      {
         return new NeckJointName[0];
      }
   }


   public static void main(String[] args) throws Exception
   {
      PlannerToolboxDataSetTest test = new PlannerToolboxDataSetTest();

      VISUALIZE = true;
      test.setup();
//      test.runAssertionsOnDataSet(DataSetName._20190626_Plank);
//      test.runAssertionsOnDataSet(DataSetName._20171215_214730_CinderBlockField);
//      test.runAssertionsOnDataSet(DataSetName._20171026_131304_PlanarRegion_Ramp_2Story_UnitTest);
//      test.runAssertionsOnDataSet(DataSetName._20171215_220523_SteppingStones);
//      test.runAssertionsOnDataSet(DataSetName._20171215_214801_StairsUpDown);
      test.runAssertionsOnDataSets(dataSetsToTest);

      ThreadTools.sleepForever();
      test.tearDown();
      PrintTools.info("Test passed.");
   }
}
