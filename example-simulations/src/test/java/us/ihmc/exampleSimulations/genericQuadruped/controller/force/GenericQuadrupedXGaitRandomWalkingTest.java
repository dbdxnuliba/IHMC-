package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitRandomWalkingTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

public class GenericQuadrupedXGaitRandomWalkingTest extends QuadrupedXGaitRandomWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   @Disabled
   @Test// timeout = 500000
   public void testExtremeRandomWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testExtremeRandomWalking();
   }

   @Override
   @Test// timeout = 1200000
   public void testWalkingRandomly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingRandomly();
   }

   @Override
   @Test// timeout = 860000
   public void testWalkingAtRandomSpeedsWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingAtRandomSpeedsWithStops();
   }

   @Override
   @Test// timeout = 1200000
   public void testWalkingRandomVelocitiesStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingRandomVelocitiesStoppingAndTurning();
   }
}
