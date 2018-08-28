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

      int currentStep = 0;
      int numberOfEndEffectors = 4;
      DenseMatrixBool previousContactState = new DenseMatrixBool(1, numberOfEndEffectors);
      DenseMatrix64F stepsNumberCounter = new DenseMatrix64F(1,numberOfEndEffectors);

      for (RobotStateCartesian robotStateCartesianIter : incomingMessage.getPoints()){
         //System.out.println(iter); // first message
         //System.out.println(robotStateCartesianIter.getEeContact()); // first message
         //System.out.println(robotStateCartesianIter.getEeMotion()); // first message
         towrCartesianStatesToFill.setBaseLinearTrajectoryWorldFrame(iter, 0, robotStateCartesianIter.getBase().getPose().getPosition().getX());
         towrCartesianStatesToFill.setBaseLinearTrajectoryWorldFrame(iter, 1, robotStateCartesianIter.getBase().getPose().getPosition().getY());
         towrCartesianStatesToFill.setBaseLinearTrajectoryWorldFrame(iter, 2, robotStateCartesianIter.getBase().getPose().getPosition().getZ());
         iter ++;
         for(LegIndex legIdx :LegIndex.values())
         {
            PrintTools.info("leg index"+ legIdx);
            if((previousContactState.get(legIdx.ordinal())==true)&&(robotStateCartesianIter.getEeContact().getBoolean(legIdx.ordinal())==false)){
               currentStep = (int)stepsNumberCounter.get(0, legIdx.ordinal());
               towrCartesianStatesToFill.setTakeOff(currentStep, legIdx, robotStateCartesianIter.getTimeFromStart().getSec()/1000.0);
            }

            if((previousContactState.get(legIdx.ordinal())==false)&&(robotStateCartesianIter.getEeContact().getBoolean(legIdx.ordinal())==true))
         {

            currentStep = (int)stepsNumberCounter.get(0, legIdx.ordinal());
            //PrintTools.info("leg ordinal: "+legIdx.ordinal());
            towrCartesianStatesToFill.setTargetFootholdWorldFrame(legIdx, currentStep, 0, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getX());
            towrCartesianStatesToFill.setTargetFootholdWorldFrame(legIdx, currentStep, 1, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getY());
            towrCartesianStatesToFill.setTargetFootholdWorldFrame(legIdx, currentStep, 2, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getZ());

            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 0, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getX() - robotStateCartesianIter.getBase().getPose().getPosition().getX());
            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 1, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getY() - robotStateCartesianIter.getBase().getPose().getPosition().getY());
            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 2, robotStateCartesianIter.getEeMotion().get(legIdx.ordinal()).getPos().getZ());

            stepsNumberCounter.set(0, legIdx.ordinal(), stepsNumberCounter.get(0, legIdx.ordinal())+1);

            towrCartesianStatesToFill.setTouchDown(currentStep, legIdx, robotStateCartesianIter.getTimeFromStart().getSec()/1000.0);
         }


         previousContactState.set(0, legIdx.ordinal(), robotStateCartesianIter.getEeContact().getBoolean(legIdx.ordinal()));
         }
      }
      towrCartesianStatesToFill.setStepsNumber(stepsNumberCounter);

      //incomingMessage.getPoints().get(0).getEeMotion().size()
      Object<StateLin3d> footPos = incomingMessage.getPoints().get(0).getEeMotion();
      //System.out.println(footPos);
      //node.spin(); // start the realtime node thread

      return towrCartesianStatesToFill;

   }

   @Override
   public List<QuadrupedTimedStepMessage> getSteps()
   {
      TowrCartesianStates towrCartesianStates = new TowrCartesianStates(200);
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
      //PrintTools.info("initial base pos TOWR:"+initial_base_pos);

      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();
      int stepsTot = 3;
      //for (LegIndex legIdx : LegIndex.values()){
         int legIdx = 0;
         DenseMatrix64F stepsTotal = towrCartesianStates.getStepsNumber();
         for(int stepCounter = 0; stepCounter< stepsTot; stepCounter++){
            double targetPositionX = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FL).get(stepCounter, 0);
            double targetPositionY = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FL).get(stepCounter, 1);
            double touchDown = towrCartesianStates.getTouchDown().get(stepCounter+1, legIdx);
            double takeOff = towrCartesianStates.getTakeOff().get(stepCounter+1, legIdx);
            steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(targetPositionX, targetPositionY, -0.012), 0.1, new TimeInterval(takeOff, touchDown)));
         }

      legIdx = 1;
      for(int stepCounter = 0; stepCounter< stepsTot; stepCounter++){
         double targetPositionX = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FR).get(stepCounter, 0);
         double targetPositionY = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FR).get(stepCounter, 1);
         double touchDown = towrCartesianStates.getTouchDown().get(stepCounter+1, legIdx);
         double takeOff = towrCartesianStates.getTakeOff().get(stepCounter+1, legIdx);
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(targetPositionX, targetPositionY, -0.012), 0.1, new TimeInterval(takeOff, touchDown)));
      }

      legIdx = 2;
      for(int stepCounter = 0; stepCounter< stepsTot; stepCounter++){
         double targetPositionX = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HL).get(stepCounter, 0);
         double targetPositionY = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HL).get(stepCounter, 1);
         double touchDown = towrCartesianStates.getTouchDown().get(stepCounter+1, legIdx);
         double takeOff = towrCartesianStates.getTakeOff().get(stepCounter+1, legIdx);
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(targetPositionX, targetPositionY, -0.012), 0.1, new TimeInterval(takeOff, touchDown)));
      }

      legIdx = 3;
      for(int stepCounter = 0; stepCounter< stepsTot; stepCounter++){
         double targetPositionX = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HR).get(stepCounter, 0);
         double targetPositionY = towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HR).get(stepCounter, 1);
         double touchDown = towrCartesianStates.getTouchDown().get(stepCounter+1, legIdx);
         double takeOff = towrCartesianStates.getTakeOff().get(stepCounter+1, legIdx);
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(targetPositionX, targetPositionY, -0.012), 0.1, new TimeInterval(takeOff, touchDown)));
      }
      //legIdx = 3;
      //for(int stepCounter = 0; stepCounter< 1; stepCounter++){
      //   int d = 1;
      //   double targetPositionX = towrCartesianStates.getFrontLeftFootPositionBaseFrame().get(stepCounter, 0);
      //   double targetPositionY = towrCartesianStates.getFrontLeftFootPositionBaseFrame().get(stepCounter, 1);
      //   double touchDown = towrCartesianStates.getTouchDown().get(stepCounter+1, legIdx);
      //   double takeOff = towrCartesianStates.getTakeOff().get(stepCounter, legIdx);
      //   steps.add(QuadrupedMessageTools
      //                   .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(targetPositionX, targetPositionY, -0.012), 0.1, new TimeInterval(takeOff, touchDown)));
      //
      //}


      //}

      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.550, -0.100, -0.012), 0.1, new TimeInterval(0.200, 0.530)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.550, 0.100, -0.012), 0.1, new TimeInterval(0.200, 0.530)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.547, 0.100, -0.012), 0.1, new TimeInterval(0.630, 0.960)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.553, -0.100, -0.012), 0.1, new TimeInterval(0.630, 0.960)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.544, -0.101, -0.012), 0.1, new TimeInterval(1.060, 1.390)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.556, 0.099, -0.012), 0.1, new TimeInterval(1.060, 1.390)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.541, 0.096, -0.012), 0.1, new TimeInterval(1.490, 1.820)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.559, -0.104, -0.012), 0.1, new TimeInterval(1.490, 1.820)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.890, 0.096, -0.000), 0.1, new TimeInterval(2.710, 3.040)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.005, -0.104, -0.000), 0.1, new TimeInterval(2.925, 3.255)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.311, -0.100, -0.000), 0.1, new TimeInterval(3.140, 3.470)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(0.380, 0.109, -0.000), 0.1, new TimeInterval(3.355, 3.685)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(1.486, 0.118, -0.000), 0.1, new TimeInterval(3.570, 3.900)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.595, -0.073, -0.000), 0.1, new TimeInterval(3.785, 4.115)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.912, -0.066, -0.000), 0.1, new TimeInterval(4.000, 4.330)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(1.023, 0.149, -0.000), 0.1, new TimeInterval(4.215, 4.545)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(2.284, 0.137, -0.000), 0.1, new TimeInterval(4.430, 4.760)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(1.404, -0.083, -0.000), 0.1, new TimeInterval(4.645, 4.975)));
      //steps.add(QuadrupedMessageTools
      //                .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(2.708, -0.080, -0.000), 0.1, new TimeInterval(4.860, 5.190)));
      return steps;
   }
}
