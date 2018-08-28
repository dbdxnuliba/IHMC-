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
   DenseMatrix64F basePositions = new DenseMatrix64F(200, 3);
   DenseMatrix64F frontLeftFootPosition = new DenseMatrix64F(10, 3);
   DenseMatrix64F frontRightFootPosition = new DenseMatrix64F(10, 3);
   DenseMatrix64F hindLeftFootPosition = new DenseMatrix64F(10, 3);
   DenseMatrix64F hindRightFootPosition = new DenseMatrix64F(10, 3);

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

   public void setBasePositions(int row, int col, double value){
      this.basePositions.set(row, col, value);
   }

   public DenseMatrix64F getBasePositions(){
      return this.basePositions;
   }

   public void setTargetFoothold(LegIndex legIndex, int row, int col, double footholdValue){
   switch (legIndex){
   case FL: this.setFrontLeftFootPosition(row, col, footholdValue);
   case FR: this.setFrontRightFootPosition(row, col, footholdValue);
   case HL: this.setHindLeftFootPosition(row, col, footholdValue);
   case HR: this.setHindRightFootPosition(row,col, footholdValue);
   }
   }

   public void setFrontLeftFootPosition(int row, int col, double value){
      this.frontLeftFootPosition.set(row, col, value);
   }

   public DenseMatrix64F getFrontLeftFootPosition(){
      return this.frontLeftFootPosition;
   }

   public void setFrontRightFootPosition(int row, int col, double value){
      this.frontRightFootPosition.set(row, col, value);
   }

   public DenseMatrix64F getFrontRightFootPosition(){
      return this.frontRightFootPosition;
   }

   public void setHindLeftFootPosition(int row, int col, double value){
      this.hindLeftFootPosition.set(row, col, value);
   }

   public DenseMatrix64F getHindLeftFootPosition(){
      return this.hindLeftFootPosition;
   }

   public void setHindRightFootPosition(int row, int col, double value){
      this.hindRightFootPosition.set(row, col, value);
   }

   public DenseMatrix64F getHindRightFootPosition(){
      return this.hindRightFootPosition;
   }
}
