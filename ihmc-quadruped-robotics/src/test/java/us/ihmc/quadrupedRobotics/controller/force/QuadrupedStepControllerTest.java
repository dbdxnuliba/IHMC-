package us.ihmc.quadrupedRobotics.controller.force;

import static org.junit.Assert.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedPlanning.input.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedStepControllerTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;
   
   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      
      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @Test// timeout = 200000
   public void testTakingAStep() throws SimulationExceededMaximumTimeException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      
      YoDouble frontLeftSolePositionX = (YoDouble) conductor.getScs().getVariable("frontLeftSolePositionX");
      double commandedStepPositionX = frontLeftSolePositionX.getDoubleValue() + 0.2;
      
      variables.getTimedStepQuadrant().set(RobotQuadrant.FRONT_LEFT);
      variables.getTimedStepGroundClearance().set(0.15);
      variables.getTimedStepDuration().set(0.6);
      variables.getTimedStepGoalPositionX().set(commandedStepPositionX);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_STEP);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 3.0));
      conductor.simulate();
      
      assertEquals("Didn't step to correct location", commandedStepPositionX, frontLeftSolePositionX.getDoubleValue(), 0.05);
   }
}
