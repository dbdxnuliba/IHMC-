package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;


import controller_msgs.msg.dds.RobotStateCartesian;
import controller_msgs.msg.dds.RobotStateCartesianTrajectory;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrCartesianStates.LegIndex;

public class QuadrupedTOWRTrajectoryConverter
{

   public void towrMessageToCartesianStateConverter(RobotStateCartesianTrajectory incomingMessage, TowrCartesianStates towrCartesianStatesToFill){

      int pointIter = 0;
      int numberOfEndEffectors = 4;
      DenseMatrixBool previousContactState = new DenseMatrixBool(1, numberOfEndEffectors);
      DenseMatrix64F stepsNumberCounter = new DenseMatrix64F(1,numberOfEndEffectors);

      for (RobotStateCartesian robotStateCartesianIter : incomingMessage.getPoints()){
         this.towrMessageToCartesianStateConverter(robotStateCartesianIter, pointIter, previousContactState, stepsNumberCounter, towrCartesianStatesToFill);
      }
      towrCartesianStatesToFill.setStepsNumber(stepsNumberCounter);
   }

   public void towrMessageToCartesianStateConverter(RobotStateCartesian robotStateCartesian, int pointNumber, DenseMatrixBool previousContactState, DenseMatrix64F stepsNumberCounter, TowrCartesianStates towrCartesianStatesToFill){

      towrCartesianStatesToFill.setBaseLinearTrajectoryWorldFrame(pointNumber, 0, robotStateCartesian.getBase().getPose().getPosition().getX());
      towrCartesianStatesToFill.setBaseLinearTrajectoryWorldFrame(pointNumber, 1, robotStateCartesian.getBase().getPose().getPosition().getY());
      towrCartesianStatesToFill.setBaseLinearTrajectoryWorldFrame(pointNumber, 2, robotStateCartesian.getBase().getPose().getPosition().getZ());
      pointNumber ++;
      for(LegIndex legIdx :LegIndex.values())
      {

         if((previousContactState.get(legIdx.ordinal())==true)&&(robotStateCartesian.getEeContact().getBoolean(legIdx.ordinal())==false)){
            int currentStep = (int)stepsNumberCounter.get(0, legIdx.ordinal());
            towrCartesianStatesToFill.setTakeOff(currentStep, legIdx, robotStateCartesian.getTimeFromStart().getSec()/1000.0);
         }

         if((previousContactState.get(legIdx.ordinal())==false)&&(robotStateCartesian.getEeContact().getBoolean(legIdx.ordinal())==true))
         {

            int currentStep = (int)stepsNumberCounter.get(0, legIdx.ordinal());
            //PrintTools.info("leg ordinal: "+legIdx.ordinal());
            towrCartesianStatesToFill.setTargetFootholdWorldFrame(legIdx, currentStep, robotStateCartesian.getEeMotion().get(legIdx.ordinal()));

            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 0, robotStateCartesian.getEeMotion().get(legIdx.ordinal()).getPos().getX() - robotStateCartesian.getBase().getPose().getPosition().getX());
            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 1, robotStateCartesian.getEeMotion().get(legIdx.ordinal()).getPos().getY() - robotStateCartesian.getBase().getPose().getPosition().getY());
            towrCartesianStatesToFill.setTargetFootholdBaseFrame(legIdx, currentStep, 2, robotStateCartesian.getEeMotion().get(legIdx.ordinal()).getPos().getZ());

            stepsNumberCounter.set(0, legIdx.ordinal(), stepsNumberCounter.get(0, legIdx.ordinal())+1);

            towrCartesianStatesToFill.setTouchDown(currentStep, legIdx, robotStateCartesian.getTimeFromStart().getSec()/1000.0);
         }


         previousContactState.set(0, legIdx.ordinal(), robotStateCartesian.getEeContact().getBoolean(legIdx.ordinal()));

      }
   }




}
