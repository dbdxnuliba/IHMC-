package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.BipedFootstepPlannerParameters;

public class AtlasFootstepPlannerParameters extends BipedFootstepPlannerParameters
{
   public AtlasFootstepPlannerParameters()
   {
      super("ihmc-open-robotics-software", "atlas/src/main/resources");
   }

   public static void main(String[] args)
   {
      new AtlasFootstepPlannerParameters();
   }
}
