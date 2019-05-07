package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerCostParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;

public class ValkyrieFootstepPlannerParameters extends BipedFootstepPlannerParameters
{
   public ValkyrieFootstepPlannerParameters()
   {
      super("ihmc-open-robotics-software", "valkyrie/src/main/resources");
   }
}
