package us.ihmc.quadrupedRobotics.planning.trajectoryConverter;

import controller_msgs.msg.dds.RobotStateCartesianTrajectory;
import controller_msgs.msg.dds.State6d;
import controller_msgs.msg.dds.State6dPubSubType;
import org.apache.commons.lang3.SystemUtils;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
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

   public static void createInitialState6dPublisher(State6d newInitialStateToPublish) throws IOException, InterruptedException
   {
      PrintTools.info("Entered publisher 3");
      //State6d state6dToPublish = new State6d();
      //initialStateToPublish.getPose().getPosition();
      while(true){
         Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "quadrupedReplanner");
         Ros2Publisher<State6d> parameter_events_pub = node.createPublisher(new State6dPubSubType(), "initial_base_state");
         parameter_events_pub.publish(newInitialStateToPublish);
         PrintTools.info("initial base pos published");
      }

      //PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ?
      //      new PeriodicRealtimeThreadSchedulerFactory(20) :
      //      new PeriodicNonRealtimeThreadSchedulerFactory();
      //RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "quadrupedReplanner", "");
      //RealtimeRos2Publisher<State6d> publisher = node.createPublisher(State6d.getPubSubType().get(), "/initial_base_state");
      //
      //publisher.publish(state6dToPublish);
   }

   public static void createInitialState6dPublisher(RealtimeRos2Node node, State6d newInitialStateToPublish) throws IOException, InterruptedException
   {
      PrintTools.info("Entered publisher 3");
      //State6d state6dToPublish = new State6d();
      //initialStateToPublish.getPose().getPosition();
      //Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "quadrupedReplanner");
      //IHMCROS2Publisher<State6d> parameter_events_pub = ROS2Tools.createPublisher(node, State6d.class, "initial_base_state");

      RealtimeRos2Publisher<State6d> publisher = node.createPublisher(State6d.getPubSubType().get(), "towr_ros2");

      //while(true){
      publisher.publish(newInitialStateToPublish);
      PrintTools.info("initial base pos published");
      //}

      //PeriodicThreadSchedulerFactory threadFactory = SystemUtils.IS_OS_LINUX ?
      //      new PeriodicRealtimeThreadSchedulerFactory(20) :
      //      new PeriodicNonRealtimeThreadSchedulerFactory();
      //RealtimeRos2Node node = new RealtimeRos2Node(PubSubImplementation.FAST_RTPS, threadFactory, "quadrupedReplanner", "");
      //RealtimeRos2Publisher<State6d> publisher = node.createPublisher(State6d.getPubSubType().get(), "/initial_base_state");
      //
      //publisher.publish(state6dToPublish);


   }


   public static void createInitialState6dPublisher(IHMCROS2Publisher<State6d> nodePublisher, State6d newInitialStateToPublish) throws IOException, InterruptedException
   {
      PrintTools.info("Entered publisher 3");
      //State6d state6dToPublish = new State6d();
      //initialStateToPublish.getPose().getPosition();
      //Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "quadrupedReplanner");
      //IHMCROS2Publisher<State6d> parameter_events_pub = ROS2Tools.createPublisher(node, State6d.class, "initial_base_state");

      //RealtimeRos2Publisher<State6d> publisher = node.createPublisher(State6d.getPubSubType().get(), "towr_ros2");

      //while(true){
      nodePublisher.publish(newInitialStateToPublish);
      PrintTools.info("initial base pos published");
   }

   public void publishInitialState6D(State6d newInitialStateToPublish)
   {
      PrintTools.info("Entered publisher 1");
      try
      {
         PrintTools.info("Entered publisher 2");
         createInitialState6dPublisher(newInitialStateToPublish);
      }
      catch (Exception e)
      {
      }
   }

   public void publishInitialState6D(RealtimeRos2Node node, State6d newInitialStateToPublish)
   {
      PrintTools.info("Entered publisher 1");
      try
      {
         PrintTools.info("Entered publisher 2");
         createInitialState6dPublisher(node, newInitialStateToPublish);
      }
      catch (Exception e)
      {
      }
   }

   public void publishInitialState6D(IHMCROS2Publisher<State6d> nodePublisher, State6d newInitialStateToPublish)
   {
      PrintTools.info("Entered publisher 1");
      try
      {
         PrintTools.info("Entered publisher 2");
         createInitialState6dPublisher(nodePublisher, newInitialStateToPublish);
      }
      catch (Exception e)
      {
      }
   }


}
