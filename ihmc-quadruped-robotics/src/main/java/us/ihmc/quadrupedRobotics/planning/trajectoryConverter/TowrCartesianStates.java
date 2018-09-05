package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;

import controller_msgs.msg.dds.RobotStateCartesian;
import controller_msgs.msg.dds.StateLin3d;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.geometry.Point3D;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.DenseMatrixBool;
import us.ihmc.commons.PrintTools;

import java.util.ArrayList;

public class TowrCartesianStates
{

   public TowrCartesianStates(int pointsNum){ setPointsNumber(pointsNum); }

   public enum LegIndex
   {
      FL, FR, HL, HR;
   }

   int centerOfMassWaypointsNumber;
   int numberOfEndEffectors = 4;
   private DenseMatrix64F centerOfMassLinearPathWorldFrame = new DenseMatrix64F(200, 3);

   private DenseMatrix64F frontLeftFootPositionWorldFrame = new DenseMatrix64F(10, 3);
   private DenseMatrix64F frontRightFootPositionWorldFrame = new DenseMatrix64F(10, 3);
   private DenseMatrix64F hindLeftFootPositionWorldFrame = new DenseMatrix64F(10, 3);
   private DenseMatrix64F hindRightFootPositionWorldFrame = new DenseMatrix64F(10, 3);

   private DenseMatrix64F frontLeftFootPositionBaseFrame = new DenseMatrix64F(10, 3);
   private DenseMatrix64F frontRightFootPositionBaseFrame = new DenseMatrix64F(10, 3);
   private DenseMatrix64F hindLeftFootPositionBaseFrame = new DenseMatrix64F(10, 3);
   private DenseMatrix64F hindRightFootPositionBaseFrame = new DenseMatrix64F(10, 3);

   private DenseMatrix64F numberOfSteps = new DenseMatrix64F(1,10);

   private DenseMatrix64F touchDownInstants = new DenseMatrix64F(10, numberOfEndEffectors);
   private DenseMatrix64F takeOffInstants = new DenseMatrix64F(10, numberOfEndEffectors);

   private DenseMatrix64F centerOfMassWaypointsTimeStamps = new DenseMatrix64F(200,1);

   public void setTimeStamps(int waypointCounter, double timeFromStart){
      this.centerOfMassWaypointsTimeStamps.set(waypointCounter, 0, timeFromStart);
      //PrintTools.info("time stamps "+centerOfMassWaypointsTimeStamps);
   }

   public DenseMatrix64F getTimeStamps(){
      return centerOfMassWaypointsTimeStamps;
   }

   public void setPointsNumber(int pointsNum){
      this.centerOfMassWaypointsNumber = pointsNum;
   }

   public int getPointsNumber()
   {
      int points = this.centerOfMassWaypointsNumber;
      //PrintTools.info("get points num: "+points);
      return points;
   }

   public void setStepsNumber(DenseMatrix64F stepsCounter){
      this.numberOfSteps = stepsCounter;
   }

   public void setStepsNumber(int stepsNum, LegIndex legIndex){
      this.numberOfSteps.set(0, legIndex.ordinal());
   }

   public DenseMatrix64F getStepsNumber(){return this.numberOfSteps;}

   public int getStepsNumber(LegIndex legIndex){
      return (int)this.numberOfSteps.get(0, legIndex.ordinal());
   }

   public void setTouchDown(int stepNumer, LegIndex legIndex, double timeInstant){
      this.touchDownInstants.set(stepNumer, legIndex.ordinal(), timeInstant);
   }

   public DenseMatrix64F getTouchDown(){
      return this.touchDownInstants;
   }

   public void setTakeOff(int stepNumer, LegIndex legIndex, double timeInstant){
      this.takeOffInstants.set(stepNumer, legIndex.ordinal(), timeInstant);
   }

   public DenseMatrix64F getTakeOff(){
      return this.takeOffInstants;
   }

   public void setCenterOfMassLinearPathWorldFrame(int row, int col, double value){
      this.centerOfMassLinearPathWorldFrame.set(row, col, value);
   }

   public DenseMatrix64F getCenterOfMassLinearPathWorldFrame(){
      return this.centerOfMassLinearPathWorldFrame;
   }

   public void setTargetFootholdWorldFrame(LegIndex legIndex, int stepNumber, StateLin3d footholdPosition){
      this.setTargetFootholdWorldFrame(legIndex, stepNumber, 0, footholdPosition.getPos().getX());
      this.setTargetFootholdWorldFrame(legIndex, stepNumber, 1, footholdPosition.getPos().getY());
      this.setTargetFootholdWorldFrame(legIndex, stepNumber, 2, footholdPosition.getPos().getZ());
   }

   public void setTargetFootholdWorldFrame(LegIndex legIndex, int stepNumber, int coordinateIndex, double footholdValue){
      switch (legIndex){
      case FL: this.setFrontLeftFootPositionWorldFrame(stepNumber, coordinateIndex, footholdValue);
         break;
      case FR: this.setFrontRightFootPositionWorldFrame(stepNumber, coordinateIndex, footholdValue);
         break;
      case HL: this.setHindLeftFootPositionWorldFrame(stepNumber, coordinateIndex, footholdValue);
         break;
      case HR: this.setHindRightFootPositionWorldFrame(stepNumber,coordinateIndex, footholdValue);
         break;
      }
   }

   public DenseMatrix64F getTargetFootholdWorldFrame(LegIndex legIndex){
      DenseMatrix64F targetFootholdsWF = new DenseMatrix64F(10,3);
      switch (legIndex){
      case FL: targetFootholdsWF = this.getFrontLeftFootPositionWorldFrame();
         break;
      case FR: targetFootholdsWF = this.getFrontRightFootPositionWorldFrame();
         break;
      case HL: targetFootholdsWF = this.getHindLeftFootPositionWorldFrame();
         break;
      case HR: targetFootholdsWF = this.getHindRightFootPositionWorldFrame();
         break;
      }
      return targetFootholdsWF;
   }

   public DenseMatrix64F getFrontLeftFootPositionWorldFrame(){ return this.frontLeftFootPositionWorldFrame; }

   public DenseMatrix64F getFrontRightFootPositionWorldFrame(){return this.frontRightFootPositionWorldFrame;}

   public DenseMatrix64F getHindLeftFootPositionWorldFrame(){
      return this.hindLeftFootPositionWorldFrame;
   }

   public DenseMatrix64F getHindRightFootPositionWorldFrame(){
      return this.hindRightFootPositionWorldFrame;
   }

   public void setTargetFootholdBaseFrame(LegIndex legIndex, int stepNumber, StateLin3d footholdPosition){
      this.setTargetFootholdBaseFrame(legIndex, stepNumber, 0, footholdPosition.getPos().getX());
      this.setTargetFootholdBaseFrame(legIndex, stepNumber, 1, footholdPosition.getPos().getY());
      this.setTargetFootholdBaseFrame(legIndex, stepNumber, 2, footholdPosition.getPos().getZ());
   }

   public void setTargetFootholdBaseFrame(LegIndex legIndex, int stepNumber, int coordinateIndex, double footholdValue){
      switch (legIndex){
      case FL: this.setFrontLeftFootPositionBaseFrame(stepNumber, coordinateIndex, footholdValue);
               break;
      case FR: this.setFrontRightFootPositionBaseFrame(stepNumber, coordinateIndex, footholdValue);
               break;
      case HL: this.setHindLeftFootPositionBaseFrame(stepNumber, coordinateIndex, footholdValue);
               break;
      case HR: this.setHindRightFootPositionBaseFrame(stepNumber,coordinateIndex, footholdValue);
               break;
      }
   }

   public DenseMatrix64F getTargetFootholdBaseFrame(LegIndex legIndex){
      DenseMatrix64F targetFootholdsBF = new DenseMatrix64F(10,3);
      switch (legIndex){
      case FL: targetFootholdsBF = this.getFrontLeftFootPositionBaseFrame();
         break;
      case FR: targetFootholdsBF = this.getFrontRightFootPositionBaseFrame();
         break;
      case HL: targetFootholdsBF = this.getHindLeftFootPositionBaseFrame();
         break;
      case HR: targetFootholdsBF = this.getHindRightFootPositionBaseFrame();
         break;
      }
      return targetFootholdsBF;
   }


   public void messageToStateConverter(RobotStateCartesian robotStateCartesian, int pointNumber, DenseMatrixBool previousContactState, DenseMatrix64F stepsNumberCounter, TowrCartesianStates towrCartesianStatesToFill){

      //TowrCartesianStates towrCartesianStatesToFill = new TowrCartesianStates(200);

      towrCartesianStatesToFill.setCenterOfMassLinearPathWorldFrame(pointNumber, 0, robotStateCartesian.getBase().getPose().getPosition().getX());
      towrCartesianStatesToFill.setCenterOfMassLinearPathWorldFrame(pointNumber, 1, robotStateCartesian.getBase().getPose().getPosition().getY());
      towrCartesianStatesToFill.setCenterOfMassLinearPathWorldFrame(pointNumber, 2, robotStateCartesian.getBase().getPose().getPosition().getZ());
      pointNumber ++;
      for(LegIndex legIdx :LegIndex.values())
      {
         //PrintTools.info("leg index"+ legIdx);
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

   private void setFrontLeftFootPositionWorldFrame(int row, int col, double value){
      this.frontLeftFootPositionWorldFrame.set(row, col, value);
   }

   private void setFrontRightFootPositionWorldFrame(int row, int col, double value){
      this.frontRightFootPositionWorldFrame.set(row, col, value);
   }

   private void setHindLeftFootPositionWorldFrame(int row, int col, double value){
      this.hindLeftFootPositionWorldFrame.set(row, col, value);
   }

   private void setHindRightFootPositionWorldFrame(int row, int col, double value){
      this.hindRightFootPositionWorldFrame.set(row, col, value);
   }

   private void setFrontLeftFootPositionBaseFrame(int row, int col, double value){
      this.frontLeftFootPositionBaseFrame.set(row, col, value);
   }

   private void setFrontRightFootPositionBaseFrame(int row, int col, double value){
      this.frontRightFootPositionBaseFrame.set(row, col, value);
   }

   private void setHindLeftFootPositionBaseFrame(int row, int col, double value){
      this.hindLeftFootPositionBaseFrame.set(row, col, value);
   }

   private void setHindRightFootPositionBaseFrame(int row, int col, double value){
      this.hindRightFootPositionBaseFrame.set(row, col, value);
   }

   private DenseMatrix64F getFrontLeftFootPositionBaseFrame(){ return this.frontLeftFootPositionBaseFrame; }

   private DenseMatrix64F getFrontRightFootPositionBaseFrame(){return this.frontRightFootPositionBaseFrame;}

   private DenseMatrix64F getHindLeftFootPositionBaseFrame(){
      return this.hindLeftFootPositionBaseFrame;
   }

   private DenseMatrix64F getHindRightFootPositionBaseFrame(){
      return this.hindRightFootPositionBaseFrame;
   }
}
