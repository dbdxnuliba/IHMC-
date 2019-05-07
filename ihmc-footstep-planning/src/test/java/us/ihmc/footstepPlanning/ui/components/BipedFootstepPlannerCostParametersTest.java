package us.ihmc.footstepPlanning.ui.components;

import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.FootstepPlanningTestTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerCostParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerCostParameters;

import java.util.Random;

public class BipedFootstepPlannerCostParametersTest
{
   private final static int iters = 10;

   @Test
   public void test()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         BipedFootstepPlannerCostParametersReadOnly costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         BipedFootstepPlannerCostParameters settableParameters = new BipedFootstepPlannerCostParameters(costParameters);

         FootstepPlanningTestTools.assertCostParametersEqual(costParameters, settableParameters);

         costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         settableParameters.set(costParameters);

         FootstepPlanningTestTools.assertCostParametersEqual(costParameters, settableParameters);

         costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         settableParameters.setUseQuadraticDistanceCost(costParameters.useQuadraticDistanceCost());
         settableParameters.setUseQuadraticHeightCost(costParameters.useQuadraticHeightCost());
         settableParameters.setAStarHeuristicsWeight(costParameters.getAStarHeuristicsWeightProvider().getValue());
         settableParameters.setVisGraphWithAStarHeuristicsWeight(costParameters.getVisGraphWithAStarHeuristicsWeightProvider().getValue());
         settableParameters.setDepthFirstHeuristicsWeight(costParameters.getDepthFirstHeuristicsWeightProvider().getValue());
         settableParameters.setBodyPathBasedHeuristicsWeight(costParameters.getBodyPathBasedHeuristicsWeightProvider().getValue());
         settableParameters.setYawWeight(costParameters.getYawWeight());
         settableParameters.setForwardWeight(costParameters.getForwardWeight());
         settableParameters.setLateralWeight(costParameters.getLateralWeight());
         settableParameters.setCostPerStep(costParameters.getCostPerStep());
         settableParameters.setStepUpWeight(costParameters.getStepUpWeight());
         settableParameters.setStepDownWeight(costParameters.getStepDownWeight());
         settableParameters.setRollWeight(costParameters.getRollWeight());
         settableParameters.setPitchWeight(costParameters.getPitchWeight());

         FootstepPlanningTestTools.assertCostParametersEqual(costParameters, settableParameters);
      }
   }

}
