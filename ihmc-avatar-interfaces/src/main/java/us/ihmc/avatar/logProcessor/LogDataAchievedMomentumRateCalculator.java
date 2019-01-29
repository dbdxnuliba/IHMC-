package us.ihmc.avatar.logProcessor;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.sql.Ref;

public class LogDataAchievedMomentumRateCalculator implements LogDataProcessorFunction
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LogDataProcessorHelper helper;
   private final YoFrameVector3D yoForceLeft, yoTorqueLeft, yoForceRight, yoTorqueRight;
   private FullHumanoidRobotModel fullRobotModel;

   private final YoFrameVector3D yoModifiedAchievedMomentumRateLinear;

   public LogDataAchievedMomentumRateCalculator(LogDataProcessorHelper logDataProcessorHelper)
   {
      this.helper = logDataProcessorHelper;

      fullRobotModel = helper.getFullRobotModel();
      RigidBodyBasics leftFoot = fullRobotModel.getFoot(RobotSide.LEFT);
      yoForceLeft = helper.findYoFrameVector("DesiredExternalWrenchleftFootForce", ReferenceFrame.getWorldFrame());
      yoTorqueLeft = helper.findYoFrameVector("DesiredExternalWrenchleftFootTorque", ReferenceFrame.getWorldFrame());
      yoForceRight = helper.findYoFrameVector("DesiredExternalWrenchrightFootForce", ReferenceFrame.getWorldFrame());
      yoTorqueRight = helper.findYoFrameVector("DesiredExternalWrenchrightFootTorque", ReferenceFrame.getWorldFrame());
      yoModifiedAchievedMomentumRateLinear = new YoFrameVector3D("modifiedAchievedMomentumRateLinear",helper.getReferenceFrames().getCenterOfMassFrame(),registry);



   }
   @Override
   public void processDataAtControllerRate()
   {
      helper.update();
      fullRobotModel = helper.getFullRobotModel();
      RigidBodyBasics leftFoot = fullRobotModel.getFoot(RobotSide.LEFT);
      FrameVector3D forceLeft = new FrameVector3D(yoForceLeft);
      forceLeft.changeFrame(helper.getReferenceFrames().getCenterOfMassFrame());
      // forceLeft.changeFrame(leftFoot.getBodyFixedFrame());
      FrameVector3D torqueLeft = new FrameVector3D(yoTorqueLeft);
      //torqueLeft.changeFrame(leftFoot.getBodyFixedFrame());
      Wrench footWrenchLeft = new Wrench(leftFoot.getBodyFixedFrame(),leftFoot.getBodyFixedFrame(),forceLeft,torqueLeft);
      //footWrenchLeft.changeFrame(helper.getReferenceFrames().getCenterOfMassFrame());

      RigidBodyBasics rightFoot = fullRobotModel.getFoot(RobotSide.RIGHT);
      FrameVector3D forceRight = new FrameVector3D(yoForceRight);
      forceRight.changeFrame(helper.getReferenceFrames().getCenterOfMassFrame());
      //forceRight.changeFrame(rightFoot.getBodyFixedFrame());
      FrameVector3D torqueRight = new FrameVector3D(yoTorqueRight);
      //torqueRight.changeFrame(rightFoot.getBodyFixedFrame());
      Wrench footWrenchRight = new Wrench(rightFoot.getBodyFixedFrame(),rightFoot.getBodyFixedFrame(),forceRight,torqueRight);
      // footWrenchRight.changeFrame(helper.getReferenceFrames().getCenterOfMassFrame());

      yoModifiedAchievedMomentumRateLinear.set(forceLeft);
      yoModifiedAchievedMomentumRateLinear.add(forceRight);


      double mg = helper.getFullRobotModel().getTotalMass()*9.81;
      FrameVector3D gravityForce = new FrameVector3D();
      gravityForce.setIncludingFrame(helper.getReferenceFrames().getCenterOfMassFrame(),0,0,-mg);
      yoModifiedAchievedMomentumRateLinear.add(gravityForce);
   }

   @Override
   public void processDataAtStateEstimatorRate()
   {


   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }
}
