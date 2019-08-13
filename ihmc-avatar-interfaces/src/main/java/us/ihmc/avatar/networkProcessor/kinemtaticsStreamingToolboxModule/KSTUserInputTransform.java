package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;

public class KSTUserInputTransform
{
   private ReferenceFrame operatorMidFootZupGroundFrame;
   private ReferenceFrame robotMidFootZUpGroundFrame;

   private double operatorToRobotHeightScale = 1.0;
   private double operatorToRobotArmScale = 1.0;
   private Point3D operatorHeadPosition;

   private final ReferenceFrame headCenteredFrame = new ReferenceFrame("headCenteredFrame", ReferenceFrame.getWorldFrame())
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.getTranslation().set(operatorHeadPosition);
      }
   };

   public KSTUserInputTransform()
   {
   }

   public void setOperatorToRobotHeightScale(double operatorToRobotHeightScale)
   {
      this.operatorToRobotHeightScale = operatorToRobotHeightScale;
   }

   public void setOperatorToRobotArmScale(double operatorToRobotArmScale)
   {
      this.operatorToRobotArmScale = operatorToRobotArmScale;
   }

   public void setOperatorMidFootZupGroundFrame(ReferenceFrame operatorMidFootZupGroundFrame)
   {
      this.operatorMidFootZupGroundFrame = operatorMidFootZupGroundFrame;
   }

   public void setRobotMidFootZUpGroundFrame(ReferenceFrame robotMidFootZUpGroundFrame)
   {
      this.robotMidFootZUpGroundFrame = robotMidFootZUpGroundFrame;
   }

   public KinematicsToolboxRigidBodyCommand transformHeadInput(KinematicsToolboxRigidBodyCommand headInput)
   {
      KinematicsToolboxRigidBodyCommand transformed = new KinematicsToolboxRigidBodyCommand();
      transformed.set(headInput);

      FramePose3D desiredPose = transformed.getDesiredPose();
      desiredPose.changeFrame(operatorMidFootZupGroundFrame);
      desiredPose.setReferenceFrame(robotMidFootZUpGroundFrame);
      desiredPose.getPosition().scale(operatorToRobotHeightScale);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      operatorHeadPosition = new Point3D(desiredPose.getPosition());

      return transformed;
   }

   public KinematicsToolboxRigidBodyCommand transformHandInput(KinematicsToolboxRigidBodyCommand handInput)
   {
      KinematicsToolboxRigidBodyCommand transformed = new KinematicsToolboxRigidBodyCommand();
      transformed.set(handInput);

      FramePose3D desiredPose = transformed.getDesiredPose();
      desiredPose.changeFrame(operatorMidFootZupGroundFrame);
      desiredPose.setReferenceFrame(robotMidFootZUpGroundFrame);
      desiredPose.changeFrame(headCenteredFrame);
      desiredPose.getPosition().scale(operatorToRobotArmScale);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());

      return transformed;
   }
}
