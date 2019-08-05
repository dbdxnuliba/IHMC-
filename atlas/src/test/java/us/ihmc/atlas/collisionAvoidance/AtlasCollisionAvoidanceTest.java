package us.ihmc.atlas.collisionAvoidance;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.collisionAvoidance.AvatarCollisionAvoidanceTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasCollisionAvoidanceTest extends AvatarCollisionAvoidanceTest
{
   FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 2, 2, true, true);
   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                                                       RobotTarget.SCS,
                                                                       false,
                                                                       simulationContactPoints);

   @Test
   public void testWalkingUpOfMediumStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.35);
      super.walkUpToHighStep(stepHeights);
   }

   @Test
   public void testWalkingUpOfMediumStepWithCustomTrajectories() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.40);
      super.walkUpToHighStepWithCustomTrajectories(stepHeights, false);
   }

   @Test
   public void testWalkingUpOfHighStepWithCustomTrajectories() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.45);
      super.walkUpToHighStepWithCustomTrajectories(stepHeights, false);
   }

   @Test
   public void testWalkingUpAndDownOf2MediumStepsAndASmallStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.3);
      stepHeights.add(0.6);
      stepHeights.add(0.3);
      stepHeights.add(0.5);
      stepHeights.add(0.8);
      stepHeights.add(0.6);

      super.walkUpToHighStep(stepHeights);
   }


   @Override
   public DRCRobotModel getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }
}
