package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import controller_msgs.msg.dds.RobotStateCartesianTrajectory;
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
   
   @Override
   public List<QuadrupedTimedStepMessage> getSteps()
   {
      //QuadrupedTOWRTrajectoryConverter quadrupedTOWRTrajectoryConverter = new QuadrupedTOWRTrajectoryConverter();
      //try
      //{
      //   us.ihmc.euclid.tuple3D.Point3D base_pos = quadrupedTOWRTrajectoryConverter.listenToTowr();
      //}
      //catch (IOException e)
      //{
      //   e.printStackTrace();
      //}
      //catch (InterruptedException e)
      //{
      //   e.printStackTrace();
      //}

      String nodeName = "towr_planner";
      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, nodeName);

      RobotSide robotSide = LEFT;
      String topicName = "towr_ros2";
      TowrConfigurationMessageSubscriber robotStateCartesianTrajectoryNewMessageListener = new TowrConfigurationMessageSubscriber(robotSide);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotStateCartesianTrajectory.class, topicName, robotStateCartesianTrajectoryNewMessageListener);

      //RobotStateCartesianTrajectory robotStateCartesianTrajectoryToListen = robotStateCartesianTrajectoryNewMessageListener.pollMessage();
      //realtimeRos2Node.spin();
      us.ihmc.euclid.tuple3D.Point3D initial_base_pos = new Point3D();
      if (robotStateCartesianTrajectoryNewMessageListener.isNewTowrTrajectoryAvailable())
      {
         RobotStateCartesianTrajectory robotStateCartesianTrajectoryToListen = robotStateCartesianTrajectoryNewMessageListener.pollMessage();
         initial_base_pos = robotStateCartesianTrajectoryNewMessageListener.pollMessage().getPoints().get(0).base_.getPose().getPosition();
         PrintTools.info("initial pos:"+initial_base_pos);//ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
         //if (controller == null)
         //   continue;

         //switch (handDesiredConfiguration)
         //{
         //case OPEN:
         //   controller.requestState(GraspState.OPEN);
         //   break;
         //
         //case CLOSE:
         //   controller.requestState(GraspState.CLOSE);
         //   break;
         //
         //default:
         //   break;
         //}
      }
      //us.ihmc.euclid.tuple3D.Point3D initial_base_pos = robotStateCartesianTrajectoryNewMessageListener.pollMessage().getPoints().get(0).base_.getPose().getPosition();
      PrintTools.info("initial base pos TOWR:"+initial_base_pos);

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
