package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PeterPlanarWalkerSimulation
{
   private boolean useHeightControl = true;
   private SimulationConstructionSet scs;
   PeterPlanarWalkerSimulation() throws SimulationExceededMaximumTimeException
   {
      double simulationDT = 1e-4;
      
      PeterPlanarWalkerRobot robot = new PeterPlanarWalkerRobot();
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      
      PeterPlanarWalkerController walkerController = new PeterPlanarWalkerController(robot, simulationDT,useHeightControl);
//      FMSPeterPlanarWalkerController walkerController = new FMSPeterPlanarWalkerController(robot, simulationDT);
      robot.setController(walkerController);
    
      scs = new SimulationConstructionSet(robot);
      
      
      scs.setDT(simulationDT, 100);
      
      scs.setMaxBufferSize(32000);
      
      scs.startOnAThread();


      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 100);

      /*
      if (withPush)
      {
         walkerController.setDesiredBodyVelocityX(DESIRED_VELOCITY);
         ExternalForcePoint externalForcePoint = new ExternalForcePoint("externalForce", robot);
         externalForcePoint.setForce(PUSH_FORCE_Y, 0, 0);
         robot.getRootJoints().get(0).addExternalForcePoint(externalForcePoint);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(PUSH_DURATION);
         robot.getRootJoints().get(0).removeExternalForcePoint(externalForcePoint);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(10.0);
      }
      */

         walkerController.setDesiredBodyVelocity(1);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(4.0);
         walkerController.setDesiredBodyVelocity(0.0);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(3.0);

   }
   
   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
     new PeterPlanarWalkerSimulation();
   }
}
