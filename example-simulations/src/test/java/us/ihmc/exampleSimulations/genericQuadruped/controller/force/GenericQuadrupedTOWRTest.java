package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.SystemUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.communication.subscribers.TowrConfigurationMessageSubscriber;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedTOWRTrajectoryTest;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates.LegIndex;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.QuadrupedTOWRTrajectoryConverter;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.robotSide.RobotSide.LEFT;

public class GenericQuadrupedTOWRTest extends QuadrupedTOWRTrajectoryTest
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
      PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ? // realtime threads only work on linux
            new PeriodicRealtimeThreadSchedulerFactory(20) :           // see https://github.com/ihmcrobotics/ihmc-realtime
            new PeriodicNonRealtimeThreadSchedulerFactory();                   // to setup realtime threads
      RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "NonRealtimeRos2PublishSubscribeExample", "");
      RealtimeRos2Publisher<RobotStateCartesianTrajectory> publisher = node.createPublisher(RobotStateCartesianTrajectory.getPubSubType().get(), "towr_ros2");

      RealtimeRos2Subscription<RobotStateCartesianTrajectory> subscription = node.createQueuedSubscription(RobotStateCartesianTrajectory.getPubSubType().get(), "towr_ros2");

      System.out.println(111111);

      RobotStateCartesianTrajectory message = new RobotStateCartesianTrajectory();
      //for (int i = 0; i < 10; i++)
      //{
      //   message.setSec(i);
      //   System.out.println(message.getSec()); // first message
      //   publisher.publish(message);  // publish
      //}

      System.out.println(111111);

      RobotStateCartesianTrajectory incomingMessage = new RobotStateCartesianTrajectory();
      while (!subscription.poll(incomingMessage))
      {
         ; // just waiting for the first message
      }
      //System.out.println(incomingMessage); // first message
      int i = 1;
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
      //System.out.println(incomingMessage); // first message

      System.out.println(111111);
      TowrCartesianStates towrCartesianStatesToFill = new TowrCartesianStates(incomingMessage.getPoints().size());
      double val = incomingMessage.getPoints().get(0).base_.getPose().getPosition().getZ();
      //towrCartesianStatesToFill.setPointsNumber(incomingMessage.getPoints().size());
      int iter = 0;

      int numberOfEndEffectors = 4;
      DenseMatrixBool previousContactState = new DenseMatrixBool(1, numberOfEndEffectors);
      DenseMatrix64F stepsNumber = new DenseMatrix64F(1,numberOfEndEffectors);
      
      for (RobotStateCartesian robotStateCartesianIter : incomingMessage.getPoints()){
         System.out.println(iter); // first message
         System.out.println(robotStateCartesianIter.getEeContact()); // first message
         System.out.println(robotStateCartesianIter.getEeMotion()); // first message
         towrCartesianStatesToFill.setBasePositions(iter, 0, robotStateCartesianIter.getBase().getPose().getPosition().getX());
         towrCartesianStatesToFill.setBasePositions(iter, 1, robotStateCartesianIter.getBase().getPose().getPosition().getY());
         towrCartesianStatesToFill.setBasePositions(iter, 2, robotStateCartesianIter.getBase().getPose().getPosition().getZ());
         iter ++;

         for(LegIndex legIdx :LegIndex.values())
         {
         if((previousContactState.get(legIdx.ordinal())==false)&&(robotStateCartesianIter.getEeContact().getBoolean(legIdx.ordinal())==true))
         {
            towrCartesianStatesToFill.setTargetFoothold(legIdx, (int)stepsNumber.get(0, legIdx.ordinal()), 0, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getX());
            towrCartesianStatesToFill.setTargetFoothold(legIdx, (int)stepsNumber.get(0, legIdx.ordinal()), 1, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getY());
            towrCartesianStatesToFill.setTargetFoothold(legIdx, (int)stepsNumber.get(0, legIdx.ordinal()), 2, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getZ());
            stepsNumber.set(0, legIdx.ordinal(), stepsNumber.get(0, legIdx.ordinal())+1);
         }
         previousContactState.set(0, legIdx.ordinal(), robotStateCartesianIter.getEeContact().getBoolean(legIdx.ordinal()));
         }
      }

      //incomingMessage.getPoints().get(0).getEeMotion().size()
      Object<StateLin3d> footPos = incomingMessage.getPoints().get(0).getEeMotion();
      System.out.println(footPos);
      //node.spin(); // start the realtime node thread

      return towrCartesianStatesToFill;

   }

   @Override
   public List<QuadrupedTimedStepMessage> getSteps()
   {
      try
      {
         TowrCartesianStates towrCartesianStates = subscribeToTowrRobotStateCartesianTrajectory();
         PrintTools.info("Number of points: "+towrCartesianStates.getPointsNumber());
         PrintTools.info("Base trajectory: "+towrCartesianStates.getBasePositions());
         PrintTools.info("FL foot trajectory: "+towrCartesianStates.getFrontLeftFootPosition());
         PrintTools.info("FR foot trajectory: "+towrCartesianStates.getFrontRightFootPosition());
         PrintTools.info("HL foot trajectory: "+towrCartesianStates.getHindLeftFootPosition());
         PrintTools.info("HR foot trajectory: "+towrCartesianStates.getHindRightFootPosition());
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
