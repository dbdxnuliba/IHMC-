package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedTeleopDesiredVelocity;
import us.ihmc.communication.controllerAPI.command.Command;

public class QuadrupedTeleopDesiredVelocityCommand implements Command<QuadrupedTeleopDesiredVelocityCommand, QuadrupedTeleopDesiredVelocity>
{
   private long sequenceId;
   private double desiredXVelocity;
   private double desiredYVelocity;
   private double desiredYawVelocity;

   public QuadrupedTeleopDesiredVelocityCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      desiredXVelocity = 0.0;
      desiredYVelocity = 0.0;
      desiredYawVelocity = 0.0;
   }

   @Override
   public void setFromMessage(QuadrupedTeleopDesiredVelocity message)
   {
      this.sequenceId = message.getSequenceId();
      this.desiredXVelocity = message.getDesiredXVelocity();
      this.desiredYVelocity = message.getDesiredYVelocity();
      this.desiredYawVelocity = message.getDesiredYawVelocity();
   }

   public double getDesiredXVelocity()
   {
      return desiredXVelocity;
   }

   public double getDesiredYVelocity()
   {
      return desiredYVelocity;
   }

   public double getDesiredYawVelocity()
   {
      return desiredYawVelocity;
   }

   @Override
   public Class<QuadrupedTeleopDesiredVelocity> getMessageClass()
   {
      return QuadrupedTeleopDesiredVelocity.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return Double.isFinite(desiredXVelocity) && Double.isFinite(desiredYVelocity) && Double.isFinite(desiredYawVelocity);
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(QuadrupedTeleopDesiredVelocityCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.desiredXVelocity = other.desiredXVelocity;
      this.desiredYVelocity = other.desiredYVelocity;
      this.desiredYawVelocity = other.desiredYawVelocity;
   }
}
