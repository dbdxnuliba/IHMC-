package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public abstract class FootstepPlannerDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;
   private static final double epsilon = 1e-3;
   public static final String TESTABLE_FLAG = "testQuadrupedPlanner";
   public static final String TIMEOUT_FLAG = "quadruped_timeout";

   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Green(),
                                                                                                                   YoAppearance.DarkRed(),
                                                                                                                   YoAppearance.DarkGreen());

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = true;
   protected static boolean VERBOSE = true;

   private QuadrupedBodyPathAndFootstepPlanner planner = null;

   protected abstract FootstepPlannerType getPlannerType();

   protected abstract QuadrupedBodyPathAndFootstepPlanner createPlanner();

   @Before
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      planner = createPlanner();
   }

   @After
   public void tearDown()
   {
      planner = null;
   }

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                             {
                                                                if(!dataSet.hasPlannerInput())
                                                                   return false;
                                                                return dataSet.getPlannerInput().getBooleanFlag(TESTABLE_FLAG);
                                                             });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   @Disabled
   @Test
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if(!dataSet.hasPlannerInput())
                                                                 return false;
                                                              // only considered "in-development" if testable flag is present and false
                                                              else if(!dataSet.getPlannerInput().containsFlag(TESTABLE_FLAG))
                                                                 return false;
                                                              return !dataSet.getPlannerInput().getBooleanFlag(TESTABLE_FLAG);
                                                           });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   protected void runAssertionsOnDataset(Function<DataSet, String> dataSetTester, String datasetName)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(datasetName);
      String errorMessages = dataSetTester.apply(dataSet);
      Assert.assertTrue("Errors:" + errorMessages, errorMessages.isEmpty());
   }

   private void runAssertionsOnAllDatasets(Function<DataSet, String> dataSetTester, List<DataSet> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      List<String> failingDatasets = new ArrayList<>();
      int numbberOfTestedSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numbberOfTestedSets++;
         String errorMessagesForCurrentFile = dataSetTester.apply(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getName());
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numbberOfTestedSets;
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
         Assert.assertEquals(message, 0, numberOfFailingTests);
      }
   }

   protected String runAssertions(DataSet dataset)
   {
      ThreadTools.sleep(1000);
      packPlanningRequest(dataset);
      String errorMessage = findPlanAndAssertGoodResult(dataset);

      visualizePlan(planner.getSteps(), dataset.getPlanarRegionsList(), dataset.getPlannerInput().getStartPosition(), dataset.getPlannerInput().getGoalPosition());

      return errorMessage;
   }

   protected void packPlanningRequest(DataSet dataset)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      startPose.setPosition(plannerInput.getStartPosition());
      goalPose.setPosition(plannerInput.getGoalPosition());
      if (plannerInput.hasStartOrientation())
         startPose.setOrientation(new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0));
      if (plannerInput.hasGoalOrientation())
         goalPose.setOrientation(new Quaternion(plannerInput.getGoalYaw(), 0.0, 0.0));

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = timeMultiplier * plannerInput.getDoubleFlag(TIMEOUT_FLAG);

      QuadrupedFootstepPlannerStart start = new QuadrupedFootstepPlannerStart();
      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      planner.setStart(start);
      planner.setGoal(goal);
      planner.setPlanarRegionsList(dataset.getPlanarRegionsList());
      planner.setTimeout(timeout);
      //      planner.setHorizonLengthTopic(Double.MAX_VALUE);

      if (DEBUG)
         LogTools.info("Set planner parameters.");
   }

   private String findPlanAndAssertGoodResult(DataSet dataset)
   {
      String datasetName = dataset.getName();

      FootstepPlanningResult pathResult = planner.planPath();
      if (!pathResult.validForExecution())
         return "Path plan for " + datasetName + " is invalid.";

      FootstepPlanningResult planResult = planner.plan();
      if (!planResult.validForExecution())
         return "Footstep plan for " + datasetName + " is invalid.";

      String errorMessage = assertPlanIsValid(datasetName, planner.getSteps(), dataset.getPlannerInput().getGoalPosition(), dataset.getPlannerInput().getGoalYaw());

      ThreadTools.sleep(1000);
      return errorMessage;
   }

   private static String assertPlanIsValid(String datasetName, List<? extends QuadrupedTimedStep> plannedSteps, Point3DReadOnly goalPosition, double goalYaw)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = getFinalStepPositions(plannedSteps);

      Point3D centerPoint = new Point3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         centerPoint.add(finalSteps.get(robotQuadrant));
      }

      double nominalYaw = FootstepNode.computeNominalYaw(finalSteps.get(RobotQuadrant.FRONT_LEFT).getX(), finalSteps.get(RobotQuadrant.FRONT_LEFT).getY(),
                                                         finalSteps.get(RobotQuadrant.FRONT_RIGHT).getX(), finalSteps.get(RobotQuadrant.FRONT_RIGHT).getY(),
                                                         finalSteps.get(RobotQuadrant.HIND_LEFT).getX(), finalSteps.get(RobotQuadrant.HIND_LEFT).getY(),
                                                         finalSteps.get(RobotQuadrant.HIND_RIGHT).getX(), finalSteps.get(RobotQuadrant.HIND_RIGHT).getY());

      centerPoint.scale(0.25);

      String errorMessage = "";
      if (!goalPosition.epsilonEquals(centerPoint, epsilon))
         errorMessage = datasetName + " did not reach goal position. Made it to " + centerPoint + ", trying to get to " + goalPosition;
      if (!Double.isNaN(goalYaw))
      {
         if (!MathTools.epsilonEquals(goalYaw, nominalYaw, 0.02))
            errorMessage = datasetName + " did not reach goal yaw. Made it to " + nominalYaw + ", trying to get to " + goalYaw;
      }

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(List<? extends QuadrupedTimedStep> plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = new QuadrantDependentList<>();
      for (int i = plannedSteps.size() - 1; i >= 0; i--)
      {
         QuadrupedTimedStep step = plannedSteps.get(i);
         if (finalSteps.containsKey(step.getRobotQuadrant()))
            continue;
         else
            finalSteps.put(step.getRobotQuadrant(), step.getGoalPosition());
      }

      return finalSteps;
   }

   private void visualizePlan(List<? extends QuadrupedTimedStep> steps, PlanarRegionsList planarRegionsList, Point3DReadOnly start, Point3DReadOnly goal)
   {
      if (!VISUALIZE || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      if (planarRegionsList != null)
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegionsList, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
      scs.setGroundVisible(false);

      graphics3DObject.identity();
      graphics3DObject.translate(start);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Blue());

      graphics3DObject.identity();
      graphics3DObject.translate(goal);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Black());

      if (steps != null)
      {
         for (int i = 0; i < steps.size(); i++)
         {
            Point3DReadOnly point = steps.get(i).getGoalPosition();
            AppearanceDefinition appearanceDefinition = colorDefinitions.get(steps.get(i).getRobotQuadrant());

            graphics3DObject.identity();
            graphics3DObject.translate(point);
            graphics3DObject.addSphere(0.1, appearanceDefinition);

         }
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }
}
