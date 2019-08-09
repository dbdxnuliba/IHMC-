package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsStreamingToolboxInputCommand implements Command<KinematicsStreamingToolboxInputCommand, KinematicsStreamingToolboxInputMessage>
{
   private long sequenceId;
   private boolean controlCenterOfMass = false;
   private final KinematicsToolboxCenterOfMassCommand centerOfMassInput = new KinematicsToolboxCenterOfMassCommand();
   private final RecyclingArrayList<KinematicsToolboxRigidBodyCommand> rigidBodyInputs = new RecyclingArrayList<>(KinematicsToolboxRigidBodyCommand::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      controlCenterOfMass = false;
      centerOfMassInput.clear();
      for (int i = 0; i < rigidBodyInputs.size(); i++)
         rigidBodyInputs.get(i).clear();
      rigidBodyInputs.clear();
   }

   @Override
   public void set(KinematicsStreamingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      controlCenterOfMass = other.controlCenterOfMass;
      centerOfMassInput.set(other.centerOfMassInput);
      rigidBodyInputs.clear();
      for (int i = 0; i < other.rigidBodyInputs.size(); i++)
         rigidBodyInputs.add().set(other.rigidBodyInputs.get(i));
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
      controlCenterOfMass = message.getControlCenterOfMass();
      centerOfMassInput.setFromMessage(message.getCenterOfMassInput());
      rigidBodyInputs.clear();
      for (int i = 0; i < message.getRigidBodyInputs().size(); i++)
         rigidBodyInputs.add().set(message.getRigidBodyInputs().get(i), rigidBodyHashCodeResolver, referenceFrameResolver);
   }

   public boolean getControlCenterOfMass()
   {
      return controlCenterOfMass;
   }

   public KinematicsToolboxCenterOfMassCommand getCenterOfMassInput()
   {
      return centerOfMassInput;
   }

   public RecyclingArrayList<KinematicsToolboxRigidBodyCommand> getRigidBodyInputs()
   {
      return rigidBodyInputs;
   }

   @Override
   public Class<KinematicsStreamingToolboxInputMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return controlCenterOfMass || !rigidBodyInputs.isEmpty();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
