package us.ihmc.communication.ros2;

import java.io.IOException;

import org.junit.Test;

import std_msgs.msg.dds.Int32;
import std_msgs.msg.dds.Int32PubSubType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2ObjectPublisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.interfaces.IHMCInterfaces;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.RealtimeRos2Subscription;

public class ROS2ObjectCommunicatorTest
{
   @SuppressWarnings("unchecked")
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testROS2ObjectCommunicator()
   {
      NetClassList netClassList = new NetClassList();
      netClassList.registerPacketClass(Int32.class);
      String topicName = NetworkPorts.CONTROLLER_PORT.getName();

      if (!IHMCInterfaces.contains(Int32.class))
         throw new AssertionError("Add Int32 to IHMCInterfaces!");

      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, topicName, ROS2Tools.RUNTIME_EXCEPTION);

      ROS2ObjectPublisher<Int32> publisher = ROS2Tools.createPublisher(realtimeRos2Node, IHMCInterfaces.getPubSubType(Int32.class), topicName,
                                                                       ROS2Tools.RUNTIME_EXCEPTION);
      Int32 unpackingMessage = (Int32) ROS2Tools.createMessage(Int32.class, ROS2Tools.RUNTIME_EXCEPTION);

      try
      {
         RealtimeRos2Subscription<Int32> createQueuedSubscription = realtimeRos2Node.createQueuedSubscription(new Int32PubSubType(), topicName);
         final Int32 data = new Int32();
         ThreadTools.startAThread(() -> {
            while (true)
            {
               if (createQueuedSubscription.poll(data))

                  System.out.println("Got " + data.toString());

               Thread.yield();
            }

         }, "SUbscriber");
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, IHMCInterfaces.getPubSubType(Int32.class), topicName, subscriber -> {
         ROS2Tools.popMessage(subscriber, unpackingMessage, DefaultExceptionHandler.PRINT_STACKTRACE);

         System.out.println("Consuming " + unpackingMessage);

      }, ROS2Tools.RUNTIME_EXCEPTION);

      for (int i = 0; i < 10; i++)
      {
         Int32 message = new Int32();
         message.setData(i);
         publisher.publish(message);
         System.out.println("Publishing: " + message.toString());

         ThreadTools.sleep(500);
      }
   }
}
