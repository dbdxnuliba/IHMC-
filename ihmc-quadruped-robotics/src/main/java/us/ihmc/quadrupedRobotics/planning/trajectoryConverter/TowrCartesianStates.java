package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedStepCommand;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class TowrCartesianStates
{

   public TowrCartesianStates(int pointsNum){
      setPointsNumber(pointsNum);

   }

   public enum LegIndex
   {
      FL, FR, HL, HR;
   }

   //DenseMatrix64F basePositions;
   int pointsNumber;
   int numberOfEndEffectors = 4;
   DenseMatrix64F baseLinearTrajectoryWorldFrame = new DenseMatrix64F(200, 3);

   DenseMatrix64F frontLeftFootPositionWorldFrame = new DenseMatrix64F(10, 3);
   DenseMatrix64F frontRightFootPositionWorldFrame = new DenseMatrix64F(10, 3);
   DenseMatrix64F hindLeftFootPositionWorldFrame = new DenseMatrix64F(10, 3);
   DenseMatrix64F hindRightFootPositionWorldFrame = new DenseMatrix64F(10, 3);

   DenseMatrix64F frontLeftFootPositionBaseFrame = new DenseMatrix64F(10, 3);
   DenseMatrix64F frontRightFootPositionBaseFrame = new DenseMatrix64F(10, 3);
   DenseMatrix64F hindLeftFootPositionBaseFrame = new DenseMatrix64F(10, 3);
   DenseMatrix64F hindRightFootPositionBaseFrame = new DenseMatrix64F(10, 3);

   DenseMatrix64F numberOfSteps = new DenseMatrix64F(1,10);

   DenseMatrix64F touchDownInstants = new DenseMatrix64F(10, numberOfEndEffectors);
   DenseMatrix64F takeOffInstants = new DenseMatrix64F(10, numberOfEndEffectors);

   public void setPointsNumber(int pointsNum){
      this.pointsNumber = pointsNum;
      //PrintTools.info("cartesian points: "+pointsNumber);
   }

   public int getPointsNumber()
   {
      int points = this.pointsNumber;
      PrintTools.info("get points num: "+points);
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

   public void setBaseLinearTrajectoryWorldFrame(int row, int col, double value){
      this.baseLinearTrajectoryWorldFrame.set(row, col, value);
   }

   public DenseMatrix64F getBaseLinearTrajectoryWorldFrame(){
      return this.baseLinearTrajectoryWorldFrame;
   }

   public void setTargetFootholdWorldFrame(LegIndex legIndex, int row, int col, double footholdValue){
   switch (legIndex){
   case FL: this.setFrontLeftFootPositionWorldFrame(row, col, footholdValue);
            break;
   case FR: this.setFrontRightFootPositionWorldFrame(row, col, footholdValue);
            break;
   case HL: this.setHindLeftFootPositionWorldFrame(row, col, footholdValue);
            break;
   case HR: this.setHindRightFootPositionWorldFrame(row,col, footholdValue);
            break;
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

   public DenseMatrix64F getFrontLeftFootPositionWorldFrame(){ return this.frontLeftFootPositionWorldFrame; }

   public DenseMatrix64F getFrontRightFootPositionWorldFrame(){return this.frontRightFootPositionWorldFrame;}

   public DenseMatrix64F getHindLeftFootPositionWorldFrame(){
      return this.hindLeftFootPositionWorldFrame;
   }

   public DenseMatrix64F getHindRightFootPositionWorldFrame(){
      return this.hindRightFootPositionWorldFrame;
   }

   public void setTargetFootholdBaseFrame(LegIndex legIndex, int row, int col, double footholdValue){
      switch (legIndex){
      case FL: this.setFrontLeftFootPositionBaseFrame(row, col, footholdValue);
               break;
      case FR: this.setFrontRightFootPositionBaseFrame(row, col, footholdValue);
               break;
      case HL: this.setHindLeftFootPositionBaseFrame(row, col, footholdValue);
               break;
      case HR: this.setHindRightFootPositionBaseFrame(row,col, footholdValue);
               break;
      }
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

   public DenseMatrix64F getTargetFootholdBaseFrame(LegIndex legIndex){
      DenseMatrix64F targetFootholds = new DenseMatrix64F(10,3);
      switch (legIndex){
      case FL: targetFootholds = this.getFrontLeftFootPositionBaseFrame();
         break;
      case FR: targetFootholds = this.getFrontRightFootPositionBaseFrame();
         break;
      case HL: targetFootholds = this.getHindLeftFootPositionBaseFrame();
         break;
      case HR: targetFootholds = this.getHindRightFootPositionBaseFrame();
         break;
      }
      return targetFootholds;
   }
}
