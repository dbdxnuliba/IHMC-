package us.ihmc.quadrupedPlanning;

import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedPlanarFootstepPlan;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedXGaitPlannerInterface
{
   void computeInitialPlan(QuadrupedPlanarFootstepPlan footstepPlan, RobotQuadrant initialStepQuadrant, double timeAtSoS);
   void computeOnlinePlan(QuadrupedPlanarFootstepPlan footstepPlan, double currentTime);
   void setStepSnapper(PointFootSnapper snapper);
}
