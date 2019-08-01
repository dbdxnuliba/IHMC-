package us.ihmc.commonWalkingControlModules.dynamicPlanning.stepUpPlanner;

import controller_msgs.msg.dds.StepUpPlannerErrorMessage;
import controller_msgs.msg.dds.StepUpPlannerParametersMessage;
import controller_msgs.msg.dds.StepUpPlannerRequestMessage;
import controller_msgs.msg.dds.StepUpPlannerRespondMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;

public class StepUpPlannerRequester
{
   private Ros2Publisher<StepUpPlannerParametersMessage> parametersPublisher;
   private Ros2Publisher<StepUpPlannerRequestMessage> requestPublisher;
   private StepUpPlannerRespondMessage receivedRespondMessage;
   private int numberOfExceptions = 0;
   private static final int NUMBER_OF_EXCEPTIONS_TO_PRINT = 5;
   private boolean parametersAcked = false;
   private boolean abort = false;
   private boolean respondReceived = false;

   private void acceptRespondMessage(Subscriber<StepUpPlannerRespondMessage> subscriber)
   {
      StepUpPlannerRespondMessage incomingData = subscriber.takeNextData();
      if (incomingData != null)
      {
         respondReceived = true;
         receivedRespondMessage = incomingData;
      }
      else
      {
         LogTools.warn("Received null from takeNextData()");
      }
   }

   private void acceptErrorMessage(Subscriber<StepUpPlannerErrorMessage> subscriber)
   {
      StepUpPlannerErrorMessage incomingData = subscriber.takeNextData();
      if (incomingData != null)
      {
         if (incomingData.getErrorCode() > 0)
         {
            LogTools.error("Recevied error: " + incomingData.getErrorDescriptionAsString());
            abort = true;
         }
         else
         {
            LogTools.info("Received ack for message with ID: " + incomingData.getSequenceIdReceived());
            parametersAcked = true;
         }
      }
      else
      {
         LogTools.warn("Received null from takeNextData()");
      }
   }

   public StepUpPlannerRequester()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stepUpPlanner_javaNode");

      ExceptionTools.handle(() -> parametersPublisher = ros2Node.createPublisher(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerParametersMessage.class),
                                                                                 "/us/ihmc/stepUpPlanner/parameters"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ExceptionTools.handle(() -> requestPublisher = ros2Node.createPublisher(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerRequestMessage.class),
                                                                              "/us/ihmc/stepUpPlanner/request"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ExceptionTools.handle(() -> ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerRespondMessage.class),
                                                              this::acceptRespondMessage,
                                                              "/us/ihmc/stepUpPlanner/respond"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ExceptionTools.handle(() -> ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(StepUpPlannerErrorMessage.class),
                                                              this::acceptErrorMessage,
                                                              "/us/ihmc/stepUpPlanner/errors"),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public void publishParameters(StepUpPlannerParametersMessage parametersMessage)
   {
      try
      {
         parametersAcked = false;
         respondReceived = false;
         abort = false;
         parametersPublisher.publish(parametersMessage);
      }
      catch (Exception e)
      {
         if (numberOfExceptions <= NUMBER_OF_EXCEPTIONS_TO_PRINT)
         {
            e.printStackTrace();

            if (++numberOfExceptions > NUMBER_OF_EXCEPTIONS_TO_PRINT)
            {
               LogTools.error("Stopping to print exceptions after {}.", NUMBER_OF_EXCEPTIONS_TO_PRINT);
            }
         }
      }
   }

   public boolean publishParametersAndWaitAck(StepUpPlannerParametersMessage parametersMessage)
   {
      return publishParametersAndWaitAck(parametersMessage, 500);
   }

   public boolean publishParametersAndWaitAck(StepUpPlannerParametersMessage parametersMessage, long millisecondsToSleep)
   {
      publishParameters(parametersMessage);
      int loop = 0;

      while (!areParametersAcked() && loop < 20 && !anErrorOccurred())
      {
         LogTools.info("Waiting to receive parameters ack.");
         ThreadTools.sleep(millisecondsToSleep);
         ++loop;
      }
      return areParametersAcked();
   }

   public void publishRequest(StepUpPlannerRequestMessage requestMessage)
   {
      try
      {
         abort = false;
         respondReceived = false;
         requestPublisher.publish(requestMessage);
      }
      catch (Exception e)
      {
         if (numberOfExceptions <= NUMBER_OF_EXCEPTIONS_TO_PRINT)
         {
            e.printStackTrace();

            if (++numberOfExceptions > NUMBER_OF_EXCEPTIONS_TO_PRINT)
            {
               LogTools.error("Stopping to print exceptions after {}.", NUMBER_OF_EXCEPTIONS_TO_PRINT);
            }
         }
      }
   }

   public StepUpPlannerRespondMessage getRespond()
   {
      if (!isRespondAvailable())
         return null;

      return receivedRespondMessage;
   }

   public StepUpPlannerRespondMessage getRespond(StepUpPlannerRequestMessage requestMessage)
   {
      return getRespond(requestMessage, 500);
   }

   public StepUpPlannerRespondMessage getRespond(StepUpPlannerRequestMessage requestMessage, long millisecondsToSleep)
   {

      publishRequest(requestMessage);
      int loop = 0;

      while (!isRespondAvailable() && loop < 50 && !anErrorOccurred())
      {
         LogTools.info("Waiting to receive respond.");
         ThreadTools.sleep(millisecondsToSleep);
         ++loop;
      }

      if (!isRespondAvailable())
      {
         return null;
      }

      return receivedRespondMessage;
   }

   public boolean isRespondAvailable()
   {
      return respondReceived;
   }

   public boolean areParametersAcked()
   {
      return parametersAcked;
   }

   public boolean anErrorOccurred()
   {
      return abort;
   }

}
