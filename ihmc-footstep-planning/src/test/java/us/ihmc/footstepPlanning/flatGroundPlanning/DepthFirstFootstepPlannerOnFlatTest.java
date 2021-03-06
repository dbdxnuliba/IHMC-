package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class DepthFirstFootstepPlannerOnFlatTest extends FootstepPlannerOnFlatGroundTest
{
   private YoVariableRegistry registry;
   private YoFootstepPlannerParameters parameters;
   private DepthFirstFootstepPlanner planner;

   private static final boolean visualize = false; // !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean keepUp = false;
   private static final boolean showPlannerVisualizer = false;

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
      planner.setMaximumNumberOfNodesToExpand(10000);
      planner.setTimeout(10.0);
      super.testJustStraightLine();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testATightTurn()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(10.0);
      planner.setExitAfterInitialSolution(false);
      super.testATightTurn();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testStraightLineWithInitialTurn()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(2.0);
      planner.setExitAfterInitialSolution(false);
      super.testStraightLineWithInitialTurn();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustTurnInPlace()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(2.0);
      planner.setExitAfterInitialSolution(false);
      super.testJustTurnInPlace();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 2.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = 30000)
   public void testRandomPoses()
   {
      planner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      planner.setTimeout(2.0);
      planner.setExitAfterInitialSolution(false);
      super.testRandomPoses();
   }

   @Before
   public void setupPlanner()
   {
      registry = new YoVariableRegistry("test");
      parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters());
      SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame = PlannerTools.createDefaultFootPolygons();
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      AlwaysValidNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      ConstantFootstepCost footstepCost = new ConstantFootstepCost(1.0);
      planner = new DepthFirstFootstepPlanner(parameters, snapper, nodeChecker, footstepCost, registry);
      planner.setFeetPolygons(footPolygonsInSoleFrame);
      planner.setMaximumNumberOfNodesToExpand(1000);
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

   @Override
   public boolean keepUp()
   {
      return keepUp;
   }
}
