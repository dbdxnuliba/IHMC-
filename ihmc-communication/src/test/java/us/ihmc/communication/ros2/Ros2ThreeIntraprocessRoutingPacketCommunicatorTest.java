package us.ihmc.communication.ros2;

import static org.junit.Assert.assertEquals;

import java.io.IOException;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationOptions;
import us.ihmc.communication.IHMCInterfacesKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class Ros2ThreeIntraprocessRoutingPacketCommunicatorTest
{
   private static final int NUMBER_OF_MESSAGES_TO_SEND = 20;

//   @Test(timeout = 60000)
//   public void testRealPacketCommunicatorROS2() throws IOException
//   {
//      CommunicationOptions.USE_ROS2 = true;
//      
//      performRealTest("ROS2 ");
//   }

   @Test(timeout = 60000)
   public void testIntraprocessPacketCommunicatorROS2() throws IOException
   {
      CommunicationOptions.USE_ROS2 = true;
      
      performIntraprocessTest("ROS2 ");
   }
   
//   @Test(timeout = 60000)
//   public void testRealPacketCommunicatorKryo() throws IOException
//   {
//      CommunicationOptions.USE_ROS2 = false;
//      
//      performRealTest("Kryo ");
//   }

   @Test(timeout = 60000)
   public void testIntraprocessPacketCommunicatorKryo() throws IOException
   {
      CommunicationOptions.USE_ROS2 = false;
      
      performIntraprocessTest("Kryo ");
   }
   
//   private void performRealTest(String layer) throws IOException
//   {
//      PacketCommunicator client = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.CONTROLLER_PORT,
//                                                                                       new IHMCInterfacesKryoNetClassList());
//   
//      PacketCommunicator server = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCInterfacesKryoNetClassList());
//   
//      performMessageTest(client, server, layer + " TCP/Fast-RTPS: ");
//   }

   private void performIntraprocessTest(String layer) throws IOException
   {
      PacketCommunicator one = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                       new IHMCInterfacesKryoNetClassList());

      PacketCommunicator two = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCInterfacesKryoNetClassList());
      
      PacketCommunicator three = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCInterfacesKryoNetClassList());
      
      performMessageTest(one, two, three, layer + " intraprocess: ");
   }

   private void performMessageTest(PacketCommunicator client, PacketCommunicator server, PacketCommunicator extra, String impl) throws IOException
   {
      MutableInt clientReceived = new MutableInt();
      client.attachListener(FootstepDataListMessage.class, object -> {
         PrintTools.info(impl + " client received: " + object.getSequenceId());
         clientReceived.increment();
      });
      
      MutableInt serverReceived = new MutableInt();
      server.attachListener(FootstepDataListMessage.class, object -> {
         PrintTools.info(impl + " server received: " + object.getSequenceId());
         serverReceived.increment();
      });
      
      MutableInt extraReceived = new MutableInt();
      extra.attachListener(FootstepDataListMessage.class, object -> {
         PrintTools.info(impl + " extra received: " + object.getSequenceId());
         extraReceived.increment();
      });

      client.connect();
      server.connect();
      extra.connect();
      
      ThreadTools.sleep(10);

      for (int i = 0; i < NUMBER_OF_MESSAGES_TO_SEND; i++)
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         RecyclingArrayListPubSub<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
         
         footstepDataListMessage.setSequenceId(i);
         for (int j = 0; j < 10; j++)
         {
            FootstepDataMessage one = footstepDataList.add();
            one.getLocation().set(0.1 * i * j, 0.2 * i * j, 0.3 * i * j);
         }
         
         server.send(footstepDataListMessage);

         ThreadTools.sleep(10);
      }
      
      assertEquals("client did not receive all messages", NUMBER_OF_MESSAGES_TO_SEND, clientReceived.intValue(), 0);
      assertEquals("extra did not receive all messages", NUMBER_OF_MESSAGES_TO_SEND, extraReceived.intValue(), 0);
      assertEquals("server should not receive messages", 0, serverReceived.intValue(), 0);
   }
}
