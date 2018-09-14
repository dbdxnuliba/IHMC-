package us.ihmc.ros2.atlasMock;


import controller_msgs.msg.dds.State6d;

import controller_msgs.msg.dds.State6dPubSubType;
import org.apache.commons.lang3.SystemUtils;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.subscribers.*;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.*;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;
import java.util.Date;

public class State6dRealTimePubSubExample
{
   double val = 0.0;

   public static double publishTopic(IHMCRealtimeROS2Publisher<State6d> publisher) throws IOException
   {

      System.out.println(111111);

      State6d state6dToPublish = new State6d();
      double val = 1.07;
      state6dToPublish.getPose().getPosition().setX(val);
      publisher.publish(state6dToPublish);

      return val;

   }

   public static void main(String[] args)
   {
      PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ? // realtime threads only work on linux
            new PeriodicRealtimeThreadSchedulerFactory(20) :           // see https://github.com/ihmcrobotics/ihmc-realtime
            new PeriodicNonRealtimeThreadSchedulerFactory();                   // to setup realtime threads
      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "RealtimeRos2PublishSubscribeExample");
      IHMCRealtimeROS2Publisher<State6d> realtimeROS2Publisher = ROS2Tools.createPublisher(realtimeRos2Node, State6d.class, "initial_base_state");
      realtimeRos2Node.spin(); // start the realtime node thread

      State6d state6dToPublish = new State6d();
      double val = 1.07;
      state6dToPublish.getPose().getPosition().setX(val);
      realtimeROS2Publisher.publish(state6dToPublish);
      System.out.println(val);

      // Instantiate a Date object


      //PrintTools.info("ID "+date.getTime());
      try
      {

         Date initialDate = new Date();
         Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "NonRealTimeRos2Node");
         Ros2Publisher<State6d> publisher = node.createPublisher(new State6dPubSubType(), "initial_base_state");
         while(true)
         {
c            Date date = new Date();
            if(date.getTime() - initialDate.getTime() > 1000.0)
            {
               PrintTools.info("ID "+date.getTime());
               publisher.publish(state6dToPublish);
               initialDate = date;
            }

            //publisher.publish(state6dToPublish);
         }
      }
      catch (Exception e)
      {
      }

   }
}
