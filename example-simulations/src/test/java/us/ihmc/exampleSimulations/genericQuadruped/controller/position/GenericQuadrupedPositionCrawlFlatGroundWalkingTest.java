package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlFlatGroundWalkingTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@Disabled
public class GenericQuadrupedPositionCrawlFlatGroundWalkingTest extends QuadrupedPositionCrawlFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingForwardFast() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardFast();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingForwardSlow() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingForwardSlow();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingBackwardsFast() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardsFast();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingBackwardsSlow() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardsSlow();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingInAForwardLeftCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInAForwardLeftCircle();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingInAForwardRightCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInAForwardRightCircle();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingInABackwardLeftCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInABackwardLeftCircle();
   }
   
   @Override
   @Test// timeout = 200000
   public void testWalkingInABackwardRightCircle() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingInABackwardRightCircle();
   }
}
