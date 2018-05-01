package us.ihmc.exampleSimulations.simpleOpenLoopRunning;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleOpenLoopPitchStabilitySimulation
{
   /**
    * Simple Open Loop Pitch Stability Simulation.
    * For gaining intuition and testing parameters for pitch stability through buoyancy effect
    * in FastRunner robots. 
    * April 25, 2018. See J.Pratt notes for some more information.
    * Trying to determine stability conditions.
    */
   public SimpleOpenLoopPitchStabilitySimulation()
   {
      SimpleOpenLoopPitchStabilityRobot robot = new SimpleOpenLoopPitchStabilityRobot();
      
      SimpleOpenLoopPitchStabilityController controller = new SimpleOpenLoopPitchStabilityController(robot);
      robot.setController(controller);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 10);
      
      scs.setSimulateNoFasterThanRealTime(true);
      
      scs.startOnAThread();
   }
   
   
   public static void main(String[] args)
   {
      new SimpleOpenLoopPitchStabilitySimulation();
   }

}
