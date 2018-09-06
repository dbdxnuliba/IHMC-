package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;


import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.SystemUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates.LegIndex;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.RealtimeRos2Publisher;
import us.ihmc.ros2.RealtimeRos2Subscription;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedTowrTrajectoryConverter
{

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
         ThreadTools.sleep(10); // just waiting for the first message
      }

      subscription.poll(incomingMessage);

      TowrCartesianStates towrCartesianStatesToFill = new TowrCartesianStates(incomingMessage.getPoints().size());
      QuadrupedTowrTrajectoryConverter quadrupedTowrTrajectoryConverter = new QuadrupedTowrTrajectoryConverter();
      quadrupedTowrTrajectoryConverter.messageToCartesianTrajectoryConverter(incomingMessage, towrCartesianStatesToFill);

      //node.spin(); // start the realtime node thread

      return towrCartesianStatesToFill;

   }

   public static void printTowrTrajectory(TowrCartesianStates towrCartesianStates)
   {
      PrintTools.info("Number of points: "+towrCartesianStates.getPointsNumber());

      String message = "Base linear trajectory: \n";
      message += "\t\t Position \t\t Velocity \t\t Acceleration\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         message += "\t" + towrCartesianStates.getCenterOfMassLinearPosition(i);
         message += "\t" + towrCartesianStates.getCenterOfMassLinearVelocity(i);
         message += "\t" + towrCartesianStates.getCenterOfMassLinearAcceleration(i) + "\n";
      }
      PrintTools.info(message);

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

   private static RobotQuadrant legIndexToRobotQuadrantConverter(LegIndex legIndex){

      switch (legIndex){
      case FL: return RobotQuadrant.FRONT_LEFT;
      case FR: return RobotQuadrant.FRONT_RIGHT;
      case HL: return RobotQuadrant.HIND_LEFT;
      case HR: return RobotQuadrant.HIND_RIGHT;
      }

      return RobotQuadrant.FRONT_LEFT;
   }

   public static List<QuadrupedTimedStepMessage> stateToTimedStepList(TowrCartesianStates towrCartesianStates)
   {
      List<QuadrupedTimedStepMessage> steps = new ArrayList<>();
      DenseMatrix64F stepsTotal = towrCartesianStates.getStepsNumber();

      for (LegIndex legIndex : LegIndex.values())
      {
         int stepsTot = (int)stepsTotal.get(legIndex.ordinal())-1;
         for (int stepCounter = 0; stepCounter < stepsTot; stepCounter++)
         {
            double targetPositionWorldFrameX = towrCartesianStates.getTargetFootholdWorldFrame(legIndex).get(stepCounter + 1, 0);
            double targetPositionWorldFrameY = towrCartesianStates.getTargetFootholdWorldFrame(legIndex).get(stepCounter + 1, 1);
            double touchDown = towrCartesianStates.getTouchDown().get(stepCounter + 1, legIndex.ordinal());
            double takeOff = towrCartesianStates.getTakeOff().get(stepCounter + 1, legIndex.ordinal());
            steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(legIndexToRobotQuadrantConverter(legIndex),
                                                                                  new Point3D(targetPositionWorldFrameX, targetPositionWorldFrameY, 0.0), 0.1,
                                                                                  new TimeInterval(takeOff, touchDown)));
         }
      }

      return steps;
   }

   public static QuadrupedTimedStepListMessage stateToTimedStepListMessage(TowrCartesianStates towrCartesianStates)
   {
      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stateToTimedStepList(towrCartesianStates), false);
      message.setCanBeDelayed(false);
      message.setIsExpressedInAbsoluteTime(false);

      return message;
   }

   public static CenterOfMassTrajectoryMessage createCenterOfMassMessage(TowrCartesianStates towrCartesianStates)
   {

      DenseMatrix64F timeStamps = towrCartesianStates.getTimeStamps();
      PrintTools.info("time stamps "+timeStamps);
      CenterOfMassTrajectoryMessage comMessage = new CenterOfMassTrajectoryMessage();
      EuclideanTrajectoryMessage euclideanTrajectoryMessage = new EuclideanTrajectoryMessage();
      int numberOfPoints = towrCartesianStates.getPointsNumber();
      for(int wayPointIterator = 0; wayPointIterator<numberOfPoints; wayPointIterator++)
      {
         EuclideanTrajectoryPointMessage euclideanTrajectoryPointMessage = new EuclideanTrajectoryPointMessage();

         euclideanTrajectoryPointMessage.getPosition().set(towrCartesianStates.getCenterOfMassLinearPosition(wayPointIterator));
         euclideanTrajectoryPointMessage.getLinearVelocity().set(towrCartesianStates.getCenterOfMassLinearVelocity(wayPointIterator));


         double currentTime = timeStamps.get(wayPointIterator);
         euclideanTrajectoryPointMessage.setTime(currentTime);
         euclideanTrajectoryPointMessage.setSequenceId(wayPointIterator);
         euclideanTrajectoryMessage.getTaskspaceTrajectoryPoints().add().set(euclideanTrajectoryPointMessage);
      }

      comMessage.getEuclideanTrajectory().set(euclideanTrajectoryMessage);
      //PrintTools.info("euclidean trajectory: "+comMessage.getEuclideanTrajectory());
      return comMessage;
   }

   public static void messageToCartesianTrajectoryConverter(RobotStateCartesianTrajectory incomingMessage, TowrCartesianStates towrCartesianStatesToFill)
   {

      int pointIter = 0;
      int numberOfEndEffectors = 4;
      DenseMatrixBool previousContactState = new DenseMatrixBool(1, numberOfEndEffectors);
      DenseMatrix64F stepsNumberCounter = new DenseMatrix64F(1,numberOfEndEffectors);

      for (RobotStateCartesian robotStateCartesianIter : incomingMessage.getPoints())
      {
         towrCartesianStatesToFill.addCenterOfMassState(robotStateCartesianIter.getBase());
         towrCartesianStatesToFill.setTimeStamps(pointIter, robotStateCartesianIter.getTimeFromStart().getSec()/1000.0);
         messageToCartesianStateConverter(robotStateCartesianIter, previousContactState, stepsNumberCounter, towrCartesianStatesToFill);
         pointIter++;
      }
      towrCartesianStatesToFill.setStepsNumber(stepsNumberCounter);
   }

   public static void messageToCartesianStateConverter(RobotStateCartesian robotStateCartesian, DenseMatrixBool previousContactState, DenseMatrix64F stepCounterPerLeg, TowrCartesianStates towrCartesianStatesToFill){

      for(LegIndex legIdx :LegIndex.values())
      {

         if((previousContactState.get(legIdx.ordinal())==true)&&(robotStateCartesian.getEeContact().getBoolean(legIdx.ordinal())==false)){
            int currentStep = (int)stepCounterPerLeg.get(0, legIdx.ordinal());
            towrCartesianStatesToFill.setTakeOff(currentStep, legIdx, robotStateCartesian.getTimeFromStart().getSec()/1000.0);
         }

         if((previousContactState.get(legIdx.ordinal())==false)&&(robotStateCartesian.getEeContact().getBoolean(legIdx.ordinal())==true))
         {

            int currentStep = (int)stepCounterPerLeg.get(0, legIdx.ordinal());
            //PrintTools.info("leg ordinal: "+legIdx.ordinal());

            towrCartesianStatesToFill.setTargetFootholdWorldFrame(legIdx, currentStep, robotStateCartesian.getEeMotion().get(legIdx.ordinal()));

            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 0, robotStateCartesian.getEeMotion().get(legIdx.ordinal()).getPos().getX() - robotStateCartesian.getBase().getPose().getPosition().getX());
            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 1, robotStateCartesian.getEeMotion().get(legIdx.ordinal()).getPos().getY() - robotStateCartesian.getBase().getPose().getPosition().getY());
            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 2, robotStateCartesian.getEeMotion().get(legIdx.ordinal()).getPos().getZ());


            towrCartesianStatesToFill.setTouchDown(currentStep, legIdx, robotStateCartesian.getTimeFromStart().getSec()/1000.0);

            stepCounterPerLeg.set(0, legIdx.ordinal(), stepCounterPerLeg.get(0, legIdx.ordinal())+1);
         }


         previousContactState.set(0, legIdx.ordinal(), robotStateCartesian.getEeContact().getBoolean(legIdx.ordinal()));

      }
   }
   
}
