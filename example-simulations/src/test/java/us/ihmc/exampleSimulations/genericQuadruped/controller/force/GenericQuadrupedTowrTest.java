package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.SystemUtils;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedTowrTrajectoryTest;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.NewTowrCartesianStates;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.QuadrupedTowrTrajectoryConverter;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates.LegIndex;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;


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

   NewTowrCartesianStates towrCartesianStates = new NewTowrCartesianStates(200);
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
      boolean useLoggedTrajectories = true;
      if(useLoggedTrajectories)
      {
         try
         {
            towrCartesianStates = QuadrupedTowrTrajectoryConverter.subscribeToTowrRobotStateCartesianTrajectory();
            QuadrupedTowrTrajectoryConverter.printTowrTrajectory(towrCartesianStates);
         }
         catch (Exception e)
         {
         }
      }else{
         //towrCartesianStates = loadLoggedTrajectories()
      }



      return QuadrupedTowrTrajectoryConverter.stateToTimedStepListMessage(towrCartesianStates);
   }

   @Override
   public CenterOfMassTrajectoryMessage getCenterOfMassTrajectoryMessage()
   {
      return QuadrupedTowrTrajectoryConverter.createCenterOfMassMessage(towrCartesianStates);
   }


}
