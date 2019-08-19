package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class KinematicsStreamingToolboxConfigurationCommand implements Command<KinematicsStreamingToolboxConfigurationCommand, KinematicsStreamingToolboxConfigurationMessage>
{
   private long sequenceId;
   private boolean requestCalibration = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      requestCalibration = false;
   }

   @Override
   public void set(KinematicsStreamingToolboxConfigurationCommand other)
   {
      sequenceId = other.sequenceId;
      requestCalibration = other.requestCalibration;
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxConfigurationMessage message)
   {
      sequenceId = message.getSequenceId();
      requestCalibration = message.getRequestCalibration();
   }

   public boolean getRequestCalibration()
   {
      return requestCalibration;
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
