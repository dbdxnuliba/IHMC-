package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@Tag("video")
public class ValkyrieObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{
   private final AdditionalSimulationContactPoints<RobotSide> footContactPoints = new AdditionalSimulationContactPoints<RobotSide>(RobotSide.values, 3, 4, true,
                                                                                                                                   false);
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false, footContactPoints);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   protected DRCRobotModel getRobotModelWithAdditionalFootContactPoints()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<RobotSide>(RobotSide.values, 5, 4, true, false);
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false, simulationContactPoints);
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @Test// timeout = 410000
   public void testTrialsTerrainZigzagHurdlesScript() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainZigzagHurdlesScript();
   }

   @Override
   @Test// timeout = 630000
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOntoAndOverSlopesSideways();
   }

   @Override
   @Test// timeout = 880000
   public void testTrialsTerrainSlopeScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }

   @Override
   @Test// timeout = 890000
   public void testTrialsTerrainSlopeScript() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainSlopeScript();
   }

   /**
    * This test doesn't make any sense. We have plenty of tests evaluating robustness to foot slip,
    * and that one adds random foot slip while stepping narrow cinder blocks. The test will often
    * fail because the foot contact points are on the edge of a cinder block.
    */
   @Override
   @Disabled
   @Test// timeout = 280000
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }
}
