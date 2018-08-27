package us.ihmc.quadrupedRobotics.communication.subscribers;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.RobotStateCartesianTrajectory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;

import java.util.concurrent.ConcurrentLinkedQueue;

public class TowrConfigurationMessageSubscriber implements NewMessageListener<RobotStateCartesianTrajectory>
{
   private final ConcurrentLinkedQueue<RobotStateCartesianTrajectory> messageQueue = new ConcurrentLinkedQueue<RobotStateCartesianTrajectory>();
   private RobotSide robotSide;

   public TowrConfigurationMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<RobotStateCartesianTrajectory> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
   }

   public void receivedPacket(RobotStateCartesianTrajectory ihmcMessage)
   {
      //if (this.robotSide == null)
      messageQueue.add(ihmcMessage);
      //else if (ihmcMessage.getRobotSide() == this.robotSide.toByte())
      //   messageQueue.add(ihmcMessage);
   }

   public RobotStateCartesianTrajectory pollMessage()
   {
      return messageQueue.poll();
   }

   public boolean isNewTowrTrajectoryAvailable()
   {
      return !messageQueue.isEmpty();
   }

   public RobotSide getSide()
   {
      return robotSide;
   }
}
