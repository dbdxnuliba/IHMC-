package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;

import controller_msgs.msg.dds.RobotStateCartesianTrajectory;
import controller_msgs.msg.dds.State6d;
import controller_msgs.msg.dds.State6dPubSubType;
import org.apache.commons.lang3.SystemUtils;
import us.ihmc.commons.PrintTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.RealtimeRos2Publisher;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;

public class TowrReplanningHandler
{

   public static void createInitialState6dPublisher() throws IOException, InterruptedException
   {
      State6d state6dToPublish = new State6d();
      state6dToPublish.getPose().getPosition().setX(1.0);
      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "quadrupedReplanner");
      Ros2Publisher<State6d> parameter_events_pub = node.createPublisher(new State6dPubSubType(), "initial_base_state");
      //while(true){
      parameter_events_pub.publish(state6dToPublish);
      PrintTools.info("initial base pos published");
      //}

      //PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ?
      //      new PeriodicRealtimeThreadSchedulerFactory(20) :
      //      new PeriodicNonRealtimeThreadSchedulerFactory();
      //RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "quadrupedReplanner", "");
      //RealtimeRos2Publisher<State6d> publisher = node.createPublisher(State6d.getPubSubType().get(), "/initial_base_state");
      //
      //publisher.publish(state6dToPublish);
      PrintTools.info("initial base pos published");

   }

   public void publishInitialState6D()
   {
      try
      {
         createInitialState6dPublisher();
      }
      catch (Exception e)
      {
      }
   }

}
