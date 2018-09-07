package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.*;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedTowrTrajectoryTest;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.QuadrupedTowrTrajectoryConverter;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

import java.util.ArrayList;

public class GenericQuadrupedTowrTest extends QuadrupedTowrTrajectoryTest
{
   @Override
   public QuadrupedInitialPositionParameters getInitialPositionParameters()
   {
      return new GenericQuadrupedDefaultInitialPosition()
      {
         @Override
         public Point3D getInitialBodyPosition()
         {
            return new Point3D(0.0, 0.0, 0.52);
         }


         @Override
         public double getHipRollAngle()
         {
            return 0.3;
         }
      };
   }

   TowrCartesianStates towrCartesianStates = new TowrCartesianStates(200);
   static ArrayList<Point3D> basePositions;
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 74.7)
   @Test(timeout = 370000)
   public void testQuadrupedTowrTrajectory() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testQuadrupedTowrTrajectory();
   }

   @ContinuousIntegrationTest(estimatedDuration = 74.7)
   @Test(timeout = 370000)
   public void testQuadrupedTowrCoMAndFeetTrajectory() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testQuadrupedTowrCoMAndFeetTrajectory();
   }

   @Override
   public Point3D getFinalPlanarPosition()
   {
      return new Point3D(1.684, 0.077, 0.0);
   }


   private final SideDependentList<RobotStateCartesianTrajectory> subscribers = new SideDependentList<>();

   @Override
   public QuadrupedTimedStepListMessage getSteps()
   {
      boolean useLoggedTrajectories = false;
      if(!useLoggedTrajectories)
      {
         try
         {
            towrCartesianStates = QuadrupedTowrTrajectoryConverter.subscribeToTowrRobotStateCartesianTrajectory();
            QuadrupedTowrTrajectoryConverter.printTowrTrajectory(towrCartesianStates);
            //QuadrupedTowrTrajectoryConverter.printDataSet(towrCartesianStates);
         }
         catch (Exception e)
         {
         }
      }else{
         towrCartesianStates = QuadrupedTowrTrajectoryConverter.loadExistingDataSet();
         //PrintTools.info("load predefined trajectory!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
         //QuadrupedTowrTrajectoryConverter.printTowrTrajectory(towrCartesianStates);
      }

      return QuadrupedTowrTrajectoryConverter.stateToTimedStepListMessage(towrCartesianStates);
   }

   @Override
   public CenterOfMassTrajectoryMessage getCenterOfMassTrajectoryMessage()
   {
      return QuadrupedTowrTrajectoryConverter.createCenterOfMassMessage(towrCartesianStates);
   }


}
