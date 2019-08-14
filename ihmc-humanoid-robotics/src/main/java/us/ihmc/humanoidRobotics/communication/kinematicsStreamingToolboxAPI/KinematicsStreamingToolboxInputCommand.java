package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsStreamingToolboxInputCommand implements Command<KinematicsStreamingToolboxInputCommand, KinematicsStreamingToolboxInputMessage>
{
   private long sequenceId;
   private final KinematicsToolboxRigidBodyCommand headInput = new KinematicsToolboxRigidBodyCommand();
   private final KinematicsToolboxRigidBodyCommand leftHandInput = new KinematicsToolboxRigidBodyCommand();
   private final KinematicsToolboxRigidBodyCommand rightHandInput = new KinematicsToolboxRigidBodyCommand();

   @Override
   public void clear()
   {
      sequenceId = 0;
      headInput.clear();
      leftHandInput.clear();
      rightHandInput.clear();
   }

   @Override
   public void set(KinematicsStreamingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      headInput.set(other.headInput);
      leftHandInput.set(other.leftHandInput);
      rightHandInput.set(other.rightHandInput);
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxInputMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsStreamingToolboxInputMessage message, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      sequenceId = message.getSequenceId();
      headInput.set(message.getHeadInput(), rigidBodyHashCodeResolver, referenceFrameResolver);
      leftHandInput.set(message.getLeftHandInput(), rigidBodyHashCodeResolver, referenceFrameResolver);
      rightHandInput.set(message.getRightHandInput(), rigidBodyHashCodeResolver, referenceFrameResolver);
   }

   public KinematicsToolboxRigidBodyCommand getHeadInput()
   {
      return headInput;
   }

   public KinematicsToolboxRigidBodyCommand getLeftHandInput()
   {
      return leftHandInput;
   }

   public KinematicsToolboxRigidBodyCommand getRightHandInput()
   {
      return rightHandInput;
   }

   @Override
   public Class<KinematicsStreamingToolboxInputMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
