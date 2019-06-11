package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CollisionManagerMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class CollisionManagerCommand implements Command<CollisionManagerCommand, CollisionManagerMessage>
{

   private float test;
   @Override
   public void set(CollisionManagerCommand other)
   {
      this.test = other.test;

   }

   public float getTest()
   {
      return test;
   }

   @Override
   public void clear()
   {
      test = (float) 0.0;
   }

   @Override
   public void setFromMessage(CollisionManagerMessage message)
   {
      test = message.getTest();

   }

   @Override
   public Class<CollisionManagerMessage> getMessageClass()
   {
      return CollisionManagerMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return 0;
   }

}
