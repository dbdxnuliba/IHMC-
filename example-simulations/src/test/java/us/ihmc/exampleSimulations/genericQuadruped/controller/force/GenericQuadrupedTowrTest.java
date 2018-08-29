package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.SystemUtils;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedTowrTrajectoryTest;
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
   static ArrayList<Point3D> basePositions;
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 74.7)
   @Test(timeout = 370000)
   public void testQuadrupedTOWRTrajectory() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testQuadrupedTOWRTrajectory();
   }

   @Override
   public Point3D getFinalPlanarPosition()
   {
      return new Point3D(1.684, 0.077, 0.0);
   }


   private final SideDependentList<RobotStateCartesianTrajectory> subscribers = new SideDependentList<>();

   public static TowrCartesianStates subscribeToTowrRobotStateCartesianTrajectory() throws IOException
   {
      PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ?
            new PeriodicRealtimeThreadSchedulerFactory(20) :
            new PeriodicNonRealtimeThreadSchedulerFactory();
      RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "RealtimeRos2Subscriber", "");
      RealtimeRos2Publisher<RobotStateCartesianTrajectory> publisher = node.createPublisher(RobotStateCartesianTrajectory.getPubSubType().get(), "towr_ros2");

      RealtimeRos2Subscription<RobotStateCartesianTrajectory> subscription = node.createQueuedSubscription(RobotStateCartesianTrajectory.getPubSubType().get(), "towr_ros2");

      RobotStateCartesianTrajectory message = new RobotStateCartesianTrajectory();

      RobotStateCartesianTrajectory incomingMessage = new RobotStateCartesianTrajectory();
      while (!subscription.poll(incomingMessage))
      {
         ; // just waiting for the first message
      }


      //while (true)
      //{
      if (subscription.poll(incomingMessage))  // poll for new messages
      {
         //System.out.println(incomingMessage);
         //i++;
      }
      else
      {
         // no available messages
      }
      //}

      TowrCartesianStates towrCartesianStatesToFill = new TowrCartesianStates(incomingMessage.getPoints().size());
      QuadrupedTowrTrajectoryConverter quadrupedTowrTrajectoryConverter = new QuadrupedTowrTrajectoryConverter();
      quadrupedTowrTrajectoryConverter.messageToCartesianTrajectoryConverter(incomingMessage, towrCartesianStatesToFill);

      //node.spin(); // start the realtime node thread

      return towrCartesianStatesToFill;

   }



   @Override
   public List<QuadrupedTimedStepMessage> getSteps()
   {
      TowrCartesianStates towrCartesianStates = new TowrCartesianStates(200);
      QuadrupedTowrTrajectoryConverter towrTrajectoryConverter = new QuadrupedTowrTrajectoryConverter();
      try
      {
         towrCartesianStates = subscribeToTowrRobotStateCartesianTrajectory();
         PrintTools.info("Number of points: "+towrCartesianStates.getPointsNumber());
         PrintTools.info("Base trajectory: "+towrCartesianStates.getBaseLinearTrajectoryWorldFrame());
         PrintTools.info("FL foot trajectory WF: "+towrCartesianStates.getFrontLeftFootPositionWorldFrame());
         PrintTools.info("FR foot trajectory WF: "+towrCartesianStates.getFrontRightFootPositionWorldFrame());
         PrintTools.info("HL foot trajectory WF: "+towrCartesianStates.getHindLeftFootPositionWorldFrame());
         PrintTools.info("HR foot trajectory WF: "+towrCartesianStates.getHindRightFootPositionWorldFrame());

         PrintTools.info("FL foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FL));
         PrintTools.info("FR foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FR));
         PrintTools.info("HL foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HL));
         PrintTools.info("HR foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HR));

         PrintTools.info("Number of steps: "+towrCartesianStates.getStepsNumber());

         PrintTools.info("Touch down: "+towrCartesianStates.getTouchDown());
         PrintTools.info("Take off: "+towrCartesianStates.getTakeOff());
      }
      catch (Exception e)
      {
      }

      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();
      towrTrajectoryConverter.stateToTimedStepMessage(towrCartesianStates, steps);

      return steps;
   }
}
