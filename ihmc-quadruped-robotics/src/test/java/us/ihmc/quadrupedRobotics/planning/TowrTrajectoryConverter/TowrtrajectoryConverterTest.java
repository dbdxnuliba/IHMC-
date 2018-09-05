package us.ihmc.quadrupedRobotics.planning.TowrTrajectoryConverter;

import controller_msgs.msg.dds.*;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
//import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
//import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
//import us.ihmc.quadrupedRobotics.controller.force.QuadrupedTowrTrajectoryTest;
//import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.QuadrupedTowrTrajectoryConverter;
//import us.ihmc.robotics.robotSide.SideDependentList;
//import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
//import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates;
//
//import java.util.ArrayList;
//import java.util.List;
//
//
//public class TowrtrajectoryConverterTest extends QuadrupedTowrTrajectoryTest
//{
//   TowrCartesianStates towrCartesianStates = new TowrCartesianStates(200);
//   static ArrayList<Point3D> basePositions;
//   @Override
//   public QuadrupedTestFactory createQuadrupedTestFactory()
//   {
//      return new GenericQuadrupedTestFactory();
//   }
//
//   @Override
//   @ContinuousIntegrationTest(estimatedDuration = 74.7)
//   @Test(timeout = 370000)
//   public void testQuadrupedTowrTrajectory() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
//   {
//      super.testQuadrupedTowrTrajectory();
//   }
//
//   @ContinuousIntegrationTest(estimatedDuration = 74.7)
//   @Test(timeout = 370000)
//   public void testQuadrupedTowrCoMAndFeetTrajectory() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
//   {
//      super.testQuadrupedTowrCoMAndFeetTrajectory();
//   }
//
//   @Override
//   public Point3D getFinalPlanarPosition()
//   {
//      return new Point3D(1.684, 0.077, 0.0);
//   }
//
//
//   private final SideDependentList<RobotStateCartesianTrajectory> subscribers = new SideDependentList<>();
//
//   @Override
//   public List<QuadrupedTimedStepMessage> getSteps()
//   {
//
//      QuadrupedTowrTrajectoryConverter towrTrajectoryConverter = new QuadrupedTowrTrajectoryConverter();
//      boolean useLoggedTrajectories = true;
//      if(useLoggedTrajectories)
//      {
//         try
//         {
//            towrCartesianStates = towrTrajectoryConverter.subscribeToTowrRobotStateCartesianTrajectory();
//            towrTrajectoryConverter.printTowrTrajectory(towrCartesianStates);
//         }
//         catch (Exception e)
//         {
//         }
//      }else{
//         //towrCartesianStates = loadLoggedTrajectories()
//      }
//
//
//      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();
//      towrTrajectoryConverter.stateToTimedStepList(towrCartesianStates, steps);
//
//      return steps;
//   }
//
//   @Override
//   public CenterOfMassTrajectoryMessage getCenterOfMassTrajectoryMessage(){
//      QuadrupedTowrTrajectoryConverter towrTrajectoryConverter = new QuadrupedTowrTrajectoryConverter();
//      return towrTrajectoryConverter.createCenterOfMassMessage(towrCartesianStates);
//   }
//
//
//}
//