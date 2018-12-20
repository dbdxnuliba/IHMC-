package us.ihmc.atlas.behaviorTests;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.WholeBodyInverseKinematicsBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasWholeBodyInverseKinematicsBehaviorTest extends WholeBodyInverseKinematicsBehaviorTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   @Test// timeout = 230000
   public void testSolvingForAHandPose() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForAHandPose();
   }

   @Override
   @Test// timeout = 230000
   public void testSolvingForBothHandPoses() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForBothHandPoses();
   }

   @Override
   @Test// timeout = 180000
   public void testSolvingForChestAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForChestAngularControl();
   }

   @Override
   @Test// timeout = 240000
   public void testSolvingForHandAngularLinearControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandAngularLinearControl();
   }

   @Override
   @Test// timeout = 250000
   public void testSolvingForHandRollConstraint() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandRollConstraint();
   }

   @Override
   @Test// timeout = 230000
   public void testSolvingForHandSelectionMatrix() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandSelectionMatrix();
   }

   @Override
   @Test// timeout = 180000
   public void testSolvingForPelvisAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForPelvisAngularControl();
   }
}
