package us.ihmc.tools.inputDevices.keyboard.linux;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class RepeatingReleasedEventsFixerTest
{
   @Test// timeout = 30000
   public void testInstallAndRemove()
   {
      RepeatingReleasedEventsFixer repeatingReleasedEventsFixer = new RepeatingReleasedEventsFixer();
      repeatingReleasedEventsFixer.install();
      
      // TODO test some dispatched events here
      
      repeatingReleasedEventsFixer.remove();
   }
}
