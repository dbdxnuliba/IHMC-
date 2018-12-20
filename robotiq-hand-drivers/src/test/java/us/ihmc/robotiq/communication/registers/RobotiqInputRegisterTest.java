package us.ihmc.robotiq.communication.registers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public abstract class RobotiqInputRegisterTest
{
   protected abstract RobotiqInputRegister getInputRegister();
   protected abstract RobotiqInputRegister getExpectedRegister();
   protected abstract byte getValueToSet();
   
   @Test// timeout = 30000
   public void testSetRegister()
   {
      RobotiqInputRegister inputRegister = getInputRegister();
      inputRegister.setRegisterValue(getValueToSet());
      
      assertTrue(getExpectedRegister().equals(inputRegister));
   }

   @Test// timeout = 30000
   public void testGetRegister()
   {
      assertEquals(getValueToSet(), getExpectedRegister().getRegisterValue());
   }

}
