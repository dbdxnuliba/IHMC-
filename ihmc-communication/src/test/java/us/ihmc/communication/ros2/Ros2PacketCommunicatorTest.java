package us.ihmc.communication.ros2;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class Ros2PacketCommunicatorTest
{
   public void testPacketCommunicator()
   {
      PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
   }
}
