package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.OutputPublisher;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.robotics.stateMachine.core.State;

public class KSTStreamingState implements State
{
   private final KSTTools tools;
   private OutputPublisher outputPublisher = m ->
   {
   };
   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();

   public KSTStreamingState(KSTTools tools)
   {
      this.tools = tools;
      configurationMessage.setJointVelocityWeight(1.0);
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      this.outputPublisher = outputPublisher;
   }

   @Override
   public void onEntry()
   {
      tools.getIKController().getDefaultGains().setMaxFeedbackAndFeedbackRate(100.0, Double.POSITIVE_INFINITY);
   }

   @Override
   public void doAction(double timeInState)
   {
      KinematicsStreamingToolboxInputCommand latestInput = tools.pollInputCommand();

      CommandInputManager ikCommandInputManager = tools.getIKCommandInputManager();

      if (latestInput != null)
      {
         if (latestInput.getControlCenterOfMass())
            ikCommandInputManager.submitCommand(latestInput.getCenterOfMassInput());
         ikCommandInputManager.submitCommands(latestInput.getRigidBodyInputs());
      }

      ikCommandInputManager.submitMessage(configurationMessage);

      tools.getIKController().updateInternal();

      outputPublisher.publish(tools.convertIKOutput());
   }

   @Override
   public void onExit()
   {
   }
}
