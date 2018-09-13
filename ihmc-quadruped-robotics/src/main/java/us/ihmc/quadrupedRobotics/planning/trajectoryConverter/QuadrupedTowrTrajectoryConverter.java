package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;


import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.SystemUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
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
      RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "quadrupedReplanner", "");
      //RealtimeRos2Publisher<RobotStateCartesianTrajectory> publisher = node.createPublisher(RobotStateCartesianTrajectory.getPubSubType().get(), "towr_ros2");

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

   public static TowrCartesianStates loadExistingDataSet()
   {
      TowrWalkDataSet1 towrWalkDataSet1 = new TowrWalkDataSet1();

      TowrCartesianStates towrCartesianStatesToFill = new TowrCartesianStates(towrWalkDataSet1.getNumberOfWayPoints());
      towrCartesianStatesToFill.setStepsNumber(towrWalkDataSet1.getNumberOfSteps());
      for (int i = 0; i< towrCartesianStatesToFill.getStepsNumber(LegIndex.FL); i++)
      {
         DenseMatrix64F times = towrWalkDataSet1.getTakeOffInstants();
         double val = towrWalkDataSet1.getTakeOffInstants().get(i,0);
         towrCartesianStatesToFill.setTakeOff(i, LegIndex.FL, val);
         towrCartesianStatesToFill.setTouchDown(i, LegIndex.FL, towrWalkDataSet1.getTouchDownInstants().get(i,0));

         double currentValueX = towrWalkDataSet1.getFrontLeftFootPositionWorldFrame().get(i,0);
         double currentValueY = towrWalkDataSet1.getFrontLeftFootPositionWorldFrame().get(i,1);
         double currentValueZ = towrWalkDataSet1.getFrontLeftFootPositionWorldFrame().get(i,2);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.FL, i, 0, currentValueX);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.FL, i, 1, currentValueY);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.FL, i, 2, currentValueZ);
      }
      for (int i = 0; i< towrCartesianStatesToFill.getStepsNumber(LegIndex.FR); i++)
      {
         towrCartesianStatesToFill.setTakeOff(i, LegIndex.FR, towrWalkDataSet1.getTakeOffInstants().get(i,1));
         towrCartesianStatesToFill.setTouchDown(i, LegIndex.FR, towrWalkDataSet1.getTouchDownInstants().get(i,1));
         double currentValueX = towrWalkDataSet1.getFrontRightFootPositionWorldFrame().get(i,0);
         double currentValueY = towrWalkDataSet1.getFrontRightFootPositionWorldFrame().get(i,1);
         double currentValueZ = towrWalkDataSet1.getFrontRightFootPositionWorldFrame().get(i,2);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.FR, i, 0, currentValueX);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.FR, i, 1, currentValueY);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.FR, i, 2, currentValueZ);
      }
      for (int i = 0; i< towrCartesianStatesToFill.getStepsNumber(LegIndex.HL); i++)
      {
         towrCartesianStatesToFill.setTakeOff(i, LegIndex.HL, towrWalkDataSet1.getTakeOffInstants().get(i,2));
         towrCartesianStatesToFill.setTouchDown(i, LegIndex.HL, towrWalkDataSet1.getTouchDownInstants().get(i,2));
         double currentValueX = towrWalkDataSet1.getHindLeftFootPositionWorldFrame().get(i,0);
         double currentValueY = towrWalkDataSet1.getHindLeftFootPositionWorldFrame().get(i,1);
         double currentValueZ = towrWalkDataSet1.getHindLeftFootPositionWorldFrame().get(i,2);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.HL, i, 0, currentValueX);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.HL, i, 1, currentValueY);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.HL, i, 2, currentValueZ);
      }
      for (int i = 0; i< towrCartesianStatesToFill.getStepsNumber(LegIndex.HR); i++)
      {
         towrCartesianStatesToFill.setTakeOff(i, LegIndex.HR, towrWalkDataSet1.getTakeOffInstants().get(i,3));
         towrCartesianStatesToFill.setTouchDown(i, LegIndex.HR, towrWalkDataSet1.getTouchDownInstants().get(i,3));
         double currentValueX = towrWalkDataSet1.getHindRightFootPositionWorldFrame().get(i,0);
         double currentValueY = towrWalkDataSet1.getHindRightFootPositionWorldFrame().get(i,1);
         double currentValueZ = towrWalkDataSet1.getHindRightFootPositionWorldFrame().get(i,2);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.HR, i, 0, currentValueX);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.HR, i, 1, currentValueY);
         towrCartesianStatesToFill.setTargetFootholdWorldFrame(LegIndex.HR, i, 2, currentValueZ);
      }

      towrCartesianStatesToFill.setPointsNumber(towrWalkDataSet1.getNumberOfWayPoints());
      for(int wayPointCounter = 0; wayPointCounter<towrWalkDataSet1.getNumberOfWayPoints(); wayPointCounter++)
      {
         towrCartesianStatesToFill.addCenterOfMassLinearPosition(towrWalkDataSet1.getCenterOfMassLinearPositions().get(wayPointCounter));
         towrCartesianStatesToFill.addCenterOfMassAngularPosition(towrWalkDataSet1.getCenterOfMassAngularOrientations().get(wayPointCounter));
         towrCartesianStatesToFill.addCenterOfMassLinearVelocity(towrWalkDataSet1.getCenterOfMassLinearVelocities().get(wayPointCounter));
         towrCartesianStatesToFill.addCenterOfMassLinearAcceleration(towrWalkDataSet1.getCenterOfMassLinearAccelerations().get(wayPointCounter));
         towrCartesianStatesToFill.addCenterOfMassAngularVelocity(towrWalkDataSet1.getCenterOfMassAngularVelocities().get(wayPointCounter));
         towrCartesianStatesToFill.addCenterOfMassAngularAcceleration(towrWalkDataSet1.getCenterOfMassAngularAccelerations().get(wayPointCounter));
      }

      return towrCartesianStatesToFill;
   }

   public static void printDataSet(TowrCartesianStates towrCartesianStates)
   {
      //PrintTools.info("Number of points: "+towrCartesianStates.getPointsNumber());

      String message = "Base linear trajectory: \n";
      //PrintTools.info("wayPointsNumber = "+towrCartesianStates.getPointsNumber());

      message += "\t wayPointsNumber = "+towrCartesianStates.getPointsNumber()+"\n";
      message += "\n";
      message += "\t stepsNumber.set("+0+", "+towrCartesianStates.getStepsNumber(LegIndex.FL)+"); \n";
      message += "\t stepsNumber.set("+1+", "+towrCartesianStates.getStepsNumber(LegIndex.FR)+"); \n";
      message += "\t stepsNumber.set("+2+", "+towrCartesianStates.getStepsNumber(LegIndex.HL)+"); \n";
      message += "\t stepsNumber.set("+3+", "+towrCartesianStates.getStepsNumber(LegIndex.HR)+"); \n";
      message += "\n";
      for (int i = 0; i< towrCartesianStates.getStepsNumber(LegIndex.FL); i++)
      {
         for (LegIndex legIdx : LegIndex.values())
         {
            message += "\t touchDownInstants.set(" + i +", "+ legIdx.ordinal() +", "+towrCartesianStates.getTouchDown().get(i,legIdx.ordinal())+");\n";
         }
      }
      message += "\n";
      for (int i = 0; i< towrCartesianStates.getStepsNumber(LegIndex.FL); i++)
      {
         for (LegIndex legIdx : LegIndex.values())
         {
            message += "\t takeOffInstants.set(" + i +", "+ legIdx.ordinal() +", "+towrCartesianStates.getTakeOff().get(i,legIdx.ordinal())+");\n";
         }
      }
      message += "\n";
      DenseMatrix64F frontLeftPositionWF = towrCartesianStates.getFrontLeftFootPositionWorldFrame();
      DenseMatrix64F frontRightPositionWF = towrCartesianStates.getFrontRightFootPositionWorldFrame();
      DenseMatrix64F hindLeftPositionWF = towrCartesianStates.getHindLeftFootPositionWorldFrame();
      DenseMatrix64F hindRightPositionWF = towrCartesianStates.getHindRightFootPositionWorldFrame();
      for (int i = 0; i< towrCartesianStates.getStepsNumber(LegIndex.FL); i++)
      {
         message += "\t frontLeftFootPositionWorldFrame.set(" + i +", "+ 0 +", "+frontLeftPositionWF.get(i,0)+");\n";
         message += "\t frontLeftFootPositionWorldFrame.set(" + i +", "+ 1 +", "+frontLeftPositionWF.get(i,1)+");\n";
         message += "\t frontLeftFootPositionWorldFrame.set(" + i +", "+ 2 +", "+frontLeftPositionWF.get(i,2)+");\n";
      }
      message += "\n";
      for (int i = 0; i< towrCartesianStates.getStepsNumber(LegIndex.FR); i++)
      {
         message += "\t frontRightFootPositionWorldFrame.set(" + i +", "+ 0 +", "+frontRightPositionWF.get(i,0)+");\n";
         message += "\t frontRightFootPositionWorldFrame.set(" + i +", "+ 1 +", "+frontRightPositionWF.get(i,1)+");\n";
         message += "\t frontRightFootPositionWorldFrame.set(" + i +", "+ 2 +", "+frontRightPositionWF.get(i,2)+");\n";
      }
      message += "\n";
      for (int i = 0; i< towrCartesianStates.getStepsNumber(LegIndex.HL); i++)
      {
         message += "\t hindLeftFootPositionWorldFrame.set(" + i +", "+ 0 +", "+hindLeftPositionWF.get(i,0)+");\n";
         message += "\t hindLeftFootPositionWorldFrame.set(" + i +", "+ 1 +", "+hindLeftPositionWF.get(i,1)+");\n";
         message += "\t hindLeftFootPositionWorldFrame.set(" + i +", "+ 2 +", "+hindLeftPositionWF.get(i,2)+");\n";
      }
      message += "\n";
      for (int i = 0; i< towrCartesianStates.getStepsNumber(LegIndex.HR); i++)
      {
         message += "\t hindRightFootPositionWorldFrame.set(" + i +", "+ 0 +", "+hindRightPositionWF.get(i,0)+");\n";
         message += "\t hindRightFootPositionWorldFrame.set(" + i +", "+ 1 +", "+hindRightPositionWF.get(i,1)+");\n";
         message += "\t hindRightFootPositionWorldFrame.set(" + i +", "+ 2 +", "+hindRightPositionWF.get(i,2)+");\n";
      }
      message += "\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         Point3DReadOnly currentPoint = towrCartesianStates.getCenterOfMassLinearPosition(i);
         message += "\t centerOfMassLinearPositions.add().set(" + currentPoint.getX()+", "+currentPoint.getY()+", "+currentPoint.getZ()+");\n";
      }
      message += "\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         Vector3DReadOnly currentVelocity = towrCartesianStates.getCenterOfMassLinearVelocity(i);
         message += "\t centerOfMassLinearVelocities.add().set(" + currentVelocity.getX()+", "+currentVelocity.getY()+", "+currentVelocity.getZ()+");\n";
      }
      message += "\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         Vector3DReadOnly currentAcceleration = towrCartesianStates.getCenterOfMassLinearAcceleration(i);
         message += "\t centerOfMassLinearAccelerations.add().set(" + currentAcceleration.getX()+", "+currentAcceleration.getY()+", "+currentAcceleration.getZ()+");\n";
      }
      message += "\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         QuaternionReadOnly currentPoint = towrCartesianStates.getCenterOfMassAngularOrientations(i);
         message += "\t centerOfMassAngularOrientations.add().set(" + currentPoint.getX()+", "+currentPoint.getY()+", "+currentPoint.getZ()+", "+currentPoint.getS()+");\n";
      }
      message += "\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         Vector3DReadOnly currentVelocity = towrCartesianStates.getCenterOfMassAngularVelocity(i);
         message += "\t centerOfMassAngularVelocities.add().set(" + currentVelocity.getX()+", "+currentVelocity.getY()+", "+currentVelocity.getZ()+");\n";
      }
      message += "\n";
      for (int i = 0; i < towrCartesianStates.getPointsNumber(); i++)
      {
         Vector3DReadOnly currentAcceleration = towrCartesianStates.getCenterOfMassAngularAcceleration(i);
         message += "\t centerOfMassAngularAccelerations.add().set(" + currentAcceleration.getX()+", "+currentAcceleration.getY()+", "+currentAcceleration.getZ()+");\n";
      }
      PrintTools.info(message);
      //
      //PrintTools.info("FL foot trajectory WF: "+towrCartesianStates.getFrontLeftFootPositionWorldFrame());
      //PrintTools.info("FR foot trajectory WF: "+towrCartesianStates.getFrontRightFootPositionWorldFrame());
      //PrintTools.info("HL foot trajectory WF: "+towrCartesianStates.getHindLeftFootPositionWorldFrame());
      //PrintTools.info("HR foot trajectory WF: "+towrCartesianStates.getHindRightFootPositionWorldFrame());
      //
      //PrintTools.info("FL foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FL));
      //PrintTools.info("FR foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.FR));
      //PrintTools.info("HL foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HL));
      //PrintTools.info("HR foot trajectory BF: "+towrCartesianStates.getTargetFootholdBaseFrame(LegIndex.HR));
      //
      //PrintTools.info("Number of steps: "+towrCartesianStates.getStepsNumber());
      //
      //PrintTools.info("Touch down: "+towrCartesianStates.getTouchDown());
      //PrintTools.info("Take off: "+towrCartesianStates.getTakeOff());
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

   public static QuadrupedBodyHeightMessage createBodyHeightMessage(TowrCartesianStates towrCartesianStates)
   {
      QuadrupedBodyHeightMessage bodyHeightMessage = new QuadrupedBodyHeightMessage();
      DenseMatrix64F timeStamps = towrCartesianStates.getTimeStamps();
      EuclideanTrajectoryMessage euclideanTrajectoryMessage = new EuclideanTrajectoryMessage();
      int numberOfPoints = towrCartesianStates.getPointsNumber();
      PrintTools.info("number of points: "+numberOfPoints);

      for(int wayPointIterator = 0; wayPointIterator < numberOfPoints; wayPointIterator++)
      {
         EuclideanTrajectoryPointMessage euclideanTrajectoryPointMessage = new EuclideanTrajectoryPointMessage();

         euclideanTrajectoryPointMessage.getPosition().set(towrCartesianStates.getCenterOfMassLinearPosition(wayPointIterator));
         //euclideanTrajectoryPointMessage.getLinearVelocity().set(towrCartesianStates.getCenterOfMassLinearVelocity(wayPointIterator));

         double currentTime = timeStamps.get(wayPointIterator);
         euclideanTrajectoryPointMessage.setTime(currentTime);
         euclideanTrajectoryPointMessage.setSequenceId(wayPointIterator);
         euclideanTrajectoryMessage.getTaskspaceTrajectoryPoints().add().set(euclideanTrajectoryPointMessage);
      }
      bodyHeightMessage.getEuclideanTrajectory().set(euclideanTrajectoryMessage);

      return bodyHeightMessage;
   }

   public static CenterOfMassTrajectoryMessage createCenterOfMassMessage(TowrCartesianStates towrCartesianStates)
   {

      DenseMatrix64F timeStamps = towrCartesianStates.getTimeStamps();
      //PrintTools.info("time stamps "+timeStamps);
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
