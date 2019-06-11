package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CollisionManagerMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class CollisionManagerCommand implements Command<CollisionManagerCommand, CollisionManagerMessage>
{

   @Override
   public void set(CollisionManagerCommand other)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void clear()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setFromMessage(CollisionManagerMessage message)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public Class<CollisionManagerMessage> getMessageClass()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean isCommandValid()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public long getSequenceId()
   {
      // TODO Auto-generated method stub
      return 0;
   }

}
