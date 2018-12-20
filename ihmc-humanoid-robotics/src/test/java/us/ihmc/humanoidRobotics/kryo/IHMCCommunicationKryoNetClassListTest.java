package us.ihmc.humanoidRobotics.kryo;

import org.junit.jupiter.api.Test;
import us.ihmc.communication.kryo.KryoNetClassListTestHelper;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class IHMCCommunicationKryoNetClassListTest
{
   @Test// timeout = 30000
   public void testAllClassesRegisteredArePackets()
   {
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      KryoNetClassListTestHelper.testAllClassesRegisteredArePackets(netClassList);
   }

   @Test// timeout = 30000
   public void testAllPacketFieldsAreRegistered()
         throws InstantiationException, IllegalAccessException, NoSuchFieldException, SecurityException, IllegalArgumentException
   {
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      KryoNetClassListTestHelper.testAllPacketFieldsAreRegistered(netClassList);
   }
}
