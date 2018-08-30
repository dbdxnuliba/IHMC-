package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;


import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.qpOASES.DenseMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates.LegIndex;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedTowrTrajectoryConverter
{

   private RobotQuadrant legIndexToRobotQuadrantConverter(LegIndex legIndex){

      switch (legIndex){
      case FL: return RobotQuadrant.FRONT_LEFT;
      case FR: return RobotQuadrant.FRONT_RIGHT;
      case HL: return RobotQuadrant.HIND_LEFT;
      case HR: return RobotQuadrant.HIND_RIGHT;
      }

      return RobotQuadrant.FRONT_LEFT;
   }

   public void stateToTimedStepList(TowrCartesianStates towrCartesianStates, ArrayList<QuadrupedTimedStepMessage> stepsToPack)
   {

      DenseMatrix64F stepsTotal = towrCartesianStates.getStepsNumber();

      for (LegIndex legIndex : LegIndex.values())
      {
         int stepsTot = (int)stepsTotal.get(legIndex.ordinal()) - 1;
         for (int stepCounter = 0; stepCounter < stepsTot; stepCounter++)
         {
            double targetPositionWorldFrameX = towrCartesianStates.getTargetFootholdWorldFrame(legIndex).get(stepCounter, 0);
            double targetPositionWorldFrameY = towrCartesianStates.getTargetFootholdWorldFrame(legIndex).get(stepCounter, 1);
            double touchDown = towrCartesianStates.getTouchDown().get(stepCounter + 1, legIndex.ordinal());
            double takeOff = towrCartesianStates.getTakeOff().get(stepCounter + 1, legIndex.ordinal());
            stepsToPack.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(this.legIndexToRobotQuadrantConverter(legIndex),
                                                                                  new Point3D(targetPositionWorldFrameX, targetPositionWorldFrameY, -0.012), 0.1,
                                                                                  new TimeInterval(takeOff, touchDown)));
         }
      }

   }

   public CenterOfMassTrajectoryMessage createCenterOfMassMessage(TowrCartesianStates towrCartesianStates){

      DenseMatrix64F comPath = towrCartesianStates.getCenterOfMassLinearPathWorldFrame();
      TDoubleArrayList timeStamps = towrCartesianStates.getTimeStamps();
      //PrintTools.info("time stamps "+timeStamps);
      CenterOfMassTrajectoryMessage comMessage = new CenterOfMassTrajectoryMessage();
      EuclideanTrajectoryMessage euclideanTrajectoryMessage = new EuclideanTrajectoryMessage();
      int numberOfPoints = comPath.getNumRows();
      for(int wayPointIterator = 0; wayPointIterator<numberOfPoints; wayPointIterator++){
         EuclideanTrajectoryPointMessage euclideanTrajectoryPointMessage = new EuclideanTrajectoryPointMessage();
         Point3D comWayPoint = new Point3D(comPath.get(wayPointIterator,0),comPath.get(wayPointIterator,1),comPath.get(wayPointIterator,2));
         euclideanTrajectoryPointMessage.getPosition().set(comWayPoint);
         //double currentTime = timeStamps.get(wayPointIterator);
         //euclideanTrajectoryPointMessage.setTime(currentTime);
         euclideanTrajectoryMessage.getTaskspaceTrajectoryPoints().add();
      }

      comMessage.getEuclideanTrajectory().set(euclideanTrajectoryMessage);
      return comMessage;
   }

   public void messageToCartesianTrajectoryConverter(RobotStateCartesianTrajectory incomingMessage, TowrCartesianStates towrCartesianStatesToFill){

      int pointIter = 0;
      int numberOfEndEffectors = 4;
      DenseMatrixBool previousContactState = new DenseMatrixBool(1, numberOfEndEffectors);
      DenseMatrix64F stepsNumberCounter = new DenseMatrix64F(1,numberOfEndEffectors);

      for (RobotStateCartesian robotStateCartesianIter : incomingMessage.getPoints()){

         towrCartesianStatesToFill.setCenterOfMassLinearPathWorldFrame(pointIter, 0, robotStateCartesianIter.getBase().getPose().getPosition().getX());
         towrCartesianStatesToFill.setCenterOfMassLinearPathWorldFrame(pointIter, 1, robotStateCartesianIter.getBase().getPose().getPosition().getY());
         towrCartesianStatesToFill.setCenterOfMassLinearPathWorldFrame(pointIter, 2, robotStateCartesianIter.getBase().getPose().getPosition().getZ());
         //towrCartesianStatesToFill.setTimeStamps(pointIter, robotStateCartesianIter.getTimeFromStart().getSec()/1000.0);
         this.messageToCartesianStateConverter(robotStateCartesianIter, previousContactState, stepsNumberCounter, towrCartesianStatesToFill);
         pointIter++;
      }
      towrCartesianStatesToFill.setStepsNumber(stepsNumberCounter);
   }

   public void messageToCartesianStateConverter(RobotStateCartesian robotStateCartesian, DenseMatrixBool previousContactState, DenseMatrix64F stepCounterPerLeg, TowrCartesianStates towrCartesianStatesToFill){

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
