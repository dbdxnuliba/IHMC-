package us.ihmc.atlas.largeStepUps;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.largeStepUps.AvatarLargeStepUpsTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLargeStepUpsTest extends AvatarLargeStepUpsTest
{
   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Test
   public void testWalkingUpOfSmallStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.2);
      super.walkUpToHighStep(stepHeights);
   }

   @Test
   public void testWalkingUpOfMediumStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.3);
      super.walkUpToHighStep(stepHeights);
   }

   @Test
   public void testWalkingUpOfHighStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.45);
      super.walkUpToHighStep(stepHeights);
   }

   @Test
   public void testWalkingUpOf2MediumStepsAndASmallStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.3);
      stepHeights.add(0.6);
      stepHeights.add(0.8);

      super.walkUpToHighStep(stepHeights);
   }

   @Test
   public void testWalkingDownOfSmallStep() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.2;
      super.walkDownFromHighStep(stepHeight);
   }

   @Test
   public void testWalkingDownOfMediumStep() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.3;
      super.walkDownFromHighStep(stepHeight);
   }

   @Test
   public void testWalkingDownOfHighStep() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.5;
      super.walkDownFromHighStep(stepHeight);
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
