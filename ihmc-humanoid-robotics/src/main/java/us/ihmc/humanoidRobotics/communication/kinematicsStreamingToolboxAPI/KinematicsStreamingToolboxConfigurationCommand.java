package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class KinematicsStreamingToolboxConfigurationCommand implements Command<KinematicsStreamingToolboxConfigurationCommand, KinematicsStreamingToolboxConfigurationMessage>
{
   private long sequenceId;

   @Override
   public void clear()
   {
   }

   @Override
   public void set(KinematicsStreamingToolboxConfigurationCommand other)
   {
      sequenceId = other.sequenceId;
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxConfigurationMessage message)
   {
      sequenceId = message.getSequenceId();
   }

   @Override
   public Class<KinematicsStreamingToolboxConfigurationMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxConfigurationMessage.class;
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
