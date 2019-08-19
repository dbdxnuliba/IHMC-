package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.OutputPublisher;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;

public class KSTStreamingState implements State
{
   private final KSTTools tools;
   private OutputPublisher outputPublisher = m ->
   {
   };
   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
   private final KSTUserInputTransform userInputTransform;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager ikCommandInputManager;

   private final KinematicsToolboxRigidBodyMessage pelvisOrientationMessage = new KinematicsToolboxRigidBodyMessage();

   public KSTStreamingState(KSTTools tools)
   {
      this.tools = tools;
      userInputTransform = tools.getUserInputTransform();
      configurationMessage.setJointVelocityWeight(1.0);
      desiredFullRobotModel = tools.getDesiredFullRobotModel();
      ikCommandInputManager = tools.getIKCommandInputManager();

      pelvisOrientationMessage.setEndEffectorHashCode(desiredFullRobotModel.getPelvis().hashCode());
      pelvisOrientationMessage.getDesiredOrientationInWorld().setToZero();
      pelvisOrientationMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false));
      pelvisOrientationMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, false, ReferenceFrame.getWorldFrame()));
      pelvisOrientationMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      this.outputPublisher = outputPublisher;
   }

   @Override
   public void onEntry()
   {
      tools.getIKController().getDefaultGains().setMaxFeedbackAndFeedbackRate(100.0, Double.POSITIVE_INFINITY);
      ikCommandInputManager.submitMessage(configurationMessage);
   }

   @Override
   public void doAction(double timeInState)
   {
      KinematicsStreamingToolboxInputCommand latestInput = tools.pollInputCommand();


      if (latestInput != null)
      {
         KinematicsToolboxRigidBodyCommand headInput = latestInput.getHeadInput();
         KinematicsToolboxRigidBodyCommand leftHandInput = latestInput.getLeftHandInput();
         KinematicsToolboxRigidBodyCommand rightHandInput = latestInput.getRightHandInput();

         RigidBodyBasics head = desiredFullRobotModel.getHead();
         RigidBodyBasics leftHand = desiredFullRobotModel.getHand(RobotSide.LEFT);
         RigidBodyBasics rightHand = desiredFullRobotModel.getHand(RobotSide.RIGHT);

         if (headInput.getEndEffector() != head)
         {
            LogTools.error("Received head input that is not for the head: " + headInput.getEndEffector().getName());
            return;
         }

         if (leftHandInput.getEndEffector() != leftHand)
         {
            LogTools.error("Received left hand input that is not for the left hand: " + leftHandInput.getEndEffector().getName());
            return;
         }

         if (rightHandInput.getEndEffector() != rightHand)
         {
            LogTools.error("Received right hand input that is not for the right hand: " + rightHandInput.getEndEffector().getName());
            return;
         }

         headInput = userInputTransform.transformHeadInput(headInput);
         leftHandInput = userInputTransform.transformHandInput(RobotSide.LEFT, leftHandInput);
         rightHandInput = userInputTransform.transformHandInput(RobotSide.RIGHT, rightHandInput);

         leftHandInput.getControlFramePose().setToZero(desiredFullRobotModel.getHandControlFrame(RobotSide.LEFT));
         leftHandInput.getControlFramePose().changeFrame(desiredFullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame());
         rightHandInput.getControlFramePose().setToZero(desiredFullRobotModel.getHandControlFrame(RobotSide.RIGHT));
         rightHandInput.getControlFramePose().changeFrame(desiredFullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame());

         ikCommandInputManager.submitCommand(headInput);
         ikCommandInputManager.submitCommand(leftHandInput);
         ikCommandInputManager.submitCommand(rightHandInput);
         ikCommandInputManager.submitMessage(pelvisOrientationMessage);
      }

      tools.getIKController().updateInternal();

      outputPublisher.publish(tools.convertIKOutput());
   }

   @Override
   public void onExit()
   {
   }
}
