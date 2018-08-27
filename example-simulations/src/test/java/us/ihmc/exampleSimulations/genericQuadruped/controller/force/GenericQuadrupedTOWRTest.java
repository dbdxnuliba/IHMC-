package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.*;
import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.communication.subscribers.TowrConfigurationMessageSubscriber;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedTOWRTrajectoryTest;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.QuadrupedTOWRTrajectoryConverter;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.robotSide.RobotSide.LEFT;

public class GenericQuadrupedTOWRTest extends QuadrupedTOWRTrajectoryTest
{
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

   public static void subscribeToTowrRobotStateCartesianTrajectory() throws IOException, InterruptedException
   {
      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "Ros2ListenerExample");
      node.createSubscription(RobotStateCartesianTrajectory.getPubSubType().get(), subscriber -> {
         RobotStateCartesianTrajectory robotStateCartesianTrajectory = new RobotStateCartesianTrajectory();
         //int pointsNumber = robotStateCartesianTrajectory.getPoints().;
         //PrintTools.info("number of points: "+pointsNumber);
         if (subscriber.takeNextData(robotStateCartesianTrajectory, null)) {

            Point3D base_pos = new Point3D();
            base_pos = robotStateCartesianTrajectory.getPoints().get(0).base_.getPose().getPosition();
            //for (RobotStateCartesian robotStateCartesian : robotStateCartesianTrajectory.getPoints()){
            //   State6d base_pose = robotStateCartesian.getBase();
            //System.out.println(base_pose);
            //}
         }
      }, "towr_ros2");

      //Thread.currentThread().join(); // keep thread alive to receive more messages

   }

   @Override
   public List<QuadrupedTimedStepMessage> getSteps()
   {
      try
      {
         subscribeToTowrRobotStateCartesianTrajectory();
      }
      catch (Exception e)
      {
      }
      //PrintTools.info("initial base pos TOWR:"+initial_base_pos);

      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.550, -0.100, -0.012), 0.1, new TimeInterval(0.200, 0.530)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.550, 0.100, -0.012), 0.1, new TimeInterval(0.200, 0.530)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.547, 0.100, -0.012), 0.1, new TimeInterval(0.630, 0.960)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.553, -0.100, -0.012), 0.1, new TimeInterval(0.630, 0.960)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.544, -0.101, -0.012), 0.1, new TimeInterval(1.060, 1.390)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.556, 0.099, -0.012), 0.1, new TimeInterval(1.060, 1.390)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.541, 0.096, -0.012), 0.1, new TimeInterval(1.490, 1.820)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.559, -0.104, -0.012), 0.1, new TimeInterval(1.490, 1.820)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.890, 0.096, -0.000), 0.1, new TimeInterval(2.710, 3.040)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.005, -0.104, -0.000), 0.1, new TimeInterval(2.925, 3.255)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.311, -0.100, -0.000), 0.1, new TimeInterval(3.140, 3.470)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(0.380, 0.109, -0.000), 0.1, new TimeInterval(3.355, 3.685)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(1.486, 0.118, -0.000), 0.1, new TimeInterval(3.570, 3.900)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.595, -0.073, -0.000), 0.1, new TimeInterval(3.785, 4.115)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.912, -0.066, -0.000), 0.1, new TimeInterval(4.000, 4.330)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(1.023, 0.149, -0.000), 0.1, new TimeInterval(4.215, 4.545)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(2.284, 0.137, -0.000), 0.1, new TimeInterval(4.430, 4.760)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(1.404, -0.083, -0.000), 0.1, new TimeInterval(4.645, 4.975)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(2.708, -0.080, -0.000), 0.1, new TimeInterval(4.860, 5.190)));
      return steps;
   }
}
