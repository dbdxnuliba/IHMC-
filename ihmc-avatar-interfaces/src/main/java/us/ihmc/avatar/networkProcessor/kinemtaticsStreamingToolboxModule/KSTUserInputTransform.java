package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KSTUserInputTransform
{
   private ReferenceFrame operatorMidFootZupGroundFrame;
   private ReferenceFrame robotMidFootZUpGroundFrame;

   private double operatorToRobotHeightScale = 1.0;
   private double operatorToRobotArmScale = 1.0;
   private Point3D operatorHeadPosition;

   private final Quaternion headOrientationOffset = new Quaternion();
   private final SideDependentList<Quaternion> handOrientationOffsets = new SideDependentList<>(Quaternion::new);

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

   public void setHeadOrientationOffset(Orientation3DReadOnly offset)
   {
      headOrientationOffset.set(offset);
   }

   public void setHandOrientationOffset(RobotSide robotSide, Orientation3DReadOnly offset)
   {
      handOrientationOffsets.get(robotSide).set(offset);
   }

   public KinematicsToolboxRigidBodyCommand transformHeadInput(KinematicsToolboxRigidBodyCommand headInput)
   {
      KinematicsToolboxRigidBodyCommand transformed = new KinematicsToolboxRigidBodyCommand();
      transformed.set(headInput);

      FramePose3D desiredPose = transformed.getDesiredPose();
      desiredPose.changeFrame(operatorMidFootZupGroundFrame);
      desiredPose.setReferenceFrame(robotMidFootZUpGroundFrame);
      desiredPose.getPosition().scale(operatorToRobotHeightScale);
      desiredPose.getOrientation().appendInvertOther(headOrientationOffset);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      operatorHeadPosition = new Point3D(desiredPose.getPosition());

      return transformed;
   }

   public KinematicsToolboxRigidBodyCommand transformHandInput(RobotSide robotSide, KinematicsToolboxRigidBodyCommand handInput)
   {
      KinematicsToolboxRigidBodyCommand transformed = new KinematicsToolboxRigidBodyCommand();
      transformed.set(handInput);

      FramePose3D desiredPose = transformed.getDesiredPose();
      desiredPose.getOrientation().appendInvertOther(handOrientationOffsets.get(robotSide));
      desiredPose.changeFrame(operatorMidFootZupGroundFrame);
      desiredPose.setReferenceFrame(robotMidFootZUpGroundFrame);
      desiredPose.changeFrame(headCenteredFrame);
      desiredPose.getPosition().scale(operatorToRobotArmScale);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());

      return transformed;
   }
}
