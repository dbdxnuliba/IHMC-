package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampsTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Tag("video")
public class ValkyrieObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @Test// timeout = 500000
   public void testWalkingDownRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingDownRampWithMediumSteps();
   }

   @Override
   @Test// timeout = 500000
   public void testWalkingUpRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithMediumSteps();
   }

   @Override
   @Test// timeout = 500000
   public void testWalkingUpRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortSteps();
   }

   @Override
   @Test// timeout = 520000
   public void testWalkingUpRampWithShortStepsALittleTooHigh() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooHigh();
   }

   @Override
   @Test// timeout = 500000
   public void testWalkingUpRampWithShortStepsALittleTooLow() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooLow();
   }

   @Override
   @Test// timeout = 500000
   public void testHeightReinitialization() throws SimulationExceededMaximumTimeException
   {
      super.testHeightReinitialization();
   }

   @Override
   protected double getMaxRotationCorruption()
   {
      return 0.0;
   }
}
