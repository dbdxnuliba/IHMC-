package us.ihmc.commonWalkingControlModules.dynamicPlanning.stepUpPlanner;

import java.util.ArrayList;

import controller_msgs.msg.dds.StepUpPlannerCostWeights;
import controller_msgs.msg.dds.StepUpPlannerErrorMessage;
import controller_msgs.msg.dds.StepUpPlannerParametersMessage;
import controller_msgs.msg.dds.StepUpPlannerPhase;
import controller_msgs.msg.dds.StepUpPlannerPhaseParameters;
import controller_msgs.msg.dds.StepUpPlannerRequestMessage;
import controller_msgs.msg.dds.StepUpPlannerRespondMessage;
import controller_msgs.msg.dds.StepUpPlannerStepParameters;
import controller_msgs.msg.dds.StepUpPlannerVector2;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class StepUpPlannerRequester
{
   private Ros2Publisher<StepUpPlannerParametersMessage> parametersPublisher;
   private Ros2Publisher<StepUpPlannerRequestMessage> requestPublisher;
   private final StepUpPlannerRespondMessage receivedRespondMessage = new StepUpPlannerRespondMessage();
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
         receivedRespondMessage.set(incomingData);
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

      while (!isRespondAvailable() && loop < 100 && !anErrorOccurred())
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

   static public StepUpPlannerParametersMessage getDefaultFivePhasesParametersMessage(SteppingParameters steppingParameters, double pelvisHeightDelta,
                                                                                      double minLegLength, double maxLegLength, double footScale,
                                                                                      Vector2D leftOffset, Vector2D rightOffset)
   {
      StepUpPlannerParametersMessage msg = new StepUpPlannerParametersMessage();

      ArrayList<StepUpPlannerStepParameters> leftSteps = new ArrayList<StepUpPlannerStepParameters>();
      ArrayList<StepUpPlannerStepParameters> rightSteps = new ArrayList<StepUpPlannerStepParameters>();
      double rearOfFoot = -steppingParameters.getFootLength() / 2.0;
      double frontOfFoot = steppingParameters.getFootLength() / 2.0;
      double toeWidth = steppingParameters.getToeWidth();
      double heelWidth = steppingParameters.getFootWidth();

      for (int i = 0; i < 5; ++i)
      {
         StepUpPlannerStepParameters newStep = new StepUpPlannerStepParameters();
         StepUpPlannerVector2 newVertex = new StepUpPlannerVector2();

         newVertex.setX(frontOfFoot);
         newVertex.setY(toeWidth / 2.0);
         newStep.getFootVertices().add().set(newVertex);

         newVertex.setX(frontOfFoot);
         newVertex.setY(-toeWidth / 2.0);
         newStep.getFootVertices().add().set(newVertex);

         newVertex.setX(rearOfFoot);
         newVertex.setY(-heelWidth / 2.0);
         newStep.getFootVertices().add().set(newVertex);

         newVertex.setX(rearOfFoot);
         newVertex.setY(heelWidth / 2.0);
         newStep.getFootVertices().add().set(newVertex);

         newStep.setScale(footScale);

         newStep.getCenterOffset().setX(leftOffset.getX());
         newStep.getCenterOffset().setY(leftOffset.getY());
         leftSteps.add(new StepUpPlannerStepParameters(newStep));

         newStep.getCenterOffset().setX(rightOffset.getX());
         newStep.getCenterOffset().setY(rightOffset.getY());
         rightSteps.add(new StepUpPlannerStepParameters(newStep));
      }

      leftSteps.get(0).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(0).setState(StepUpPlannerStepParameters.STAND);

      leftSteps.get(1).setState(StepUpPlannerStepParameters.SWING);
      rightSteps.get(1).setState(StepUpPlannerStepParameters.STAND);

      leftSteps.get(2).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(2).setState(StepUpPlannerStepParameters.STAND);

      leftSteps.get(3).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(3).setState(StepUpPlannerStepParameters.SWING);

      leftSteps.get(4).setState(StepUpPlannerStepParameters.STAND);
      rightSteps.get(4).setState(StepUpPlannerStepParameters.STAND);

      for (int p = 0; p < 5; ++p)
      {
         StepUpPlannerPhaseParameters newSettings = new StepUpPlannerPhaseParameters();
         newSettings.getLeftStepParameters().set(leftSteps.get(p));
         newSettings.getRightStepParameters().set(rightSteps.get(p));
         msg.getPhasesParameters().add().set(newSettings);
      }

      msg.setPhaseLength(30);
      msg.setSolverVerbosity(1);
      msg.setMinLegLength(minLegLength);
      msg.setMaxLegLength(maxLegLength);
      msg.setIpoptLinearSolver("mumps");
      msg.setFinalStateAnticipation(0.3);
      msg.setStaticFrictionCoefficient(0.7);
      msg.setTorsionalFrictionCoefficient(0.1);

      double N = msg.getPhaseLength() * msg.getPhasesParameters().size();

      StepUpPlannerCostWeights weights = new StepUpPlannerCostWeights();

      weights.setCop(1.0 / N);
      weights.setTorques(0.1 / N);
      weights.setMaxTorques(4.0);
      weights.setControlMultipliers(0.1 / N);
      weights.setFinalControl(1.0);
      weights.setMaxControlMultiplier(0.1);
      weights.setFinalState(10.0);
      weights.setControlVariations(200.0 / N);
      weights.setDurationsDifference(5.0 / msg.getPhasesParameters().size());

      msg.getCostWeights().set(weights);

      msg.setSequenceId(1);

      msg.setSendComMessages(false);
      msg.setMaxComMessageLength(40);
      msg.setIncludeComMessages(true);

      msg.setSendPelvisHeightMessages(false);
      msg.setMaxPelvisHeightMessageLength(40);
      msg.setIncludePelvisHeightMessages(true);
      msg.setPelvisHeightDelta(pelvisHeightDelta);

      msg.setSendFootstepMessages(false);
      msg.setIncludeFootstepMessages(true);

      return msg;
   }

   static public StepUpPlannerRequestMessage getDefaultFivePhasesRequestMessage(Vector3D desiredDeltaInMidFeetCoordinates, double desiredlegLength,
                                                                                CommonHumanoidReferenceFrames referenceFrames)
   {
      StepUpPlannerRequestMessage msg = new StepUpPlannerRequestMessage();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();
      ReferenceFrame comFrame = referenceFrames.getCenterOfMassFrame();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePoint3D initialCoMPosition = new FramePoint3D(comFrame);
      initialCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());
      
      FramePoint3D desiredCoMPosition = new FramePoint3D(comFrame);
      desiredCoMPosition.changeFrame(midFeetFrame);
      desiredCoMPosition.add(desiredDeltaInMidFeetCoordinates);      
      desiredCoMPosition.changeFrame(worldFrame);

      msg.getInitialComPosition().set(initialCoMPosition);
      msg.getInitialComVelocity().setToZero();
      msg.getDesiredComPosition().set(desiredCoMPosition);
      msg.getDesiredComVelocity().setToZero();

      FrameQuaternion identityQuaternion = new FrameQuaternion();
      identityQuaternion.set(0, 0, 0, 1.0);

      msg.getPhases().clear();

      MovingReferenceFrame leftSoleFrame = referenceFrames.getSoleZUpFrame(RobotSide.LEFT);
      FramePose3D initialLeftPose = new FramePose3D(leftSoleFrame);
      initialLeftPose.changeFrame(worldFrame);
      
      FramePose3D desiredLeftPose = new FramePose3D(leftSoleFrame);
      desiredLeftPose.changeFrame(midFeetFrame);
      desiredLeftPose.getPosition().add(desiredDeltaInMidFeetCoordinates);
      desiredLeftPose.changeFrame(worldFrame);
      

      MovingReferenceFrame rightSoleFrame = referenceFrames.getSoleZUpFrame(RobotSide.RIGHT);
      FramePose3D initialRightPose = new FramePose3D(rightSoleFrame);
      initialRightPose.changeFrame(worldFrame);
      
      FramePose3D desiredRightPose = new FramePose3D(rightSoleFrame);
      desiredRightPose.changeFrame(midFeetFrame);
      desiredRightPose.getPosition().add(desiredDeltaInMidFeetCoordinates);
      desiredRightPose.changeFrame(worldFrame);


      StepUpPlannerPhase newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(initialLeftPose);
      newPhase.getRightFootPose().set(initialRightPose);
      newPhase.setMinimumDuration(1.8);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.8);

      newPhase = msg.getPhases().add();
      newPhase.getRightFootPose().set(initialRightPose);
      newPhase.setMinimumDuration(1.8);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.8);

      newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(desiredLeftPose);
      newPhase.getRightFootPose().set(initialRightPose);
      newPhase.setMinimumDuration(1.8);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.8);

      newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(desiredLeftPose);
      newPhase.setMinimumDuration(1.8);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.8);

      newPhase = msg.getPhases().add();
      newPhase.getLeftFootPose().set(desiredLeftPose);
      newPhase.getRightFootPose().set(desiredRightPose);
      newPhase.setMinimumDuration(1.4);
      newPhase.setMaximumDuration(2.0);
      newPhase.setDesiredDuration(1.4);

      msg.setDesiredLegLength(desiredlegLength);

      msg.getLeftDesiredFinalControl().getCop().setX(0.0);
      msg.getLeftDesiredFinalControl().getCop().setY(0.0);
      msg.getRightDesiredFinalControl().set(msg.getLeftDesiredFinalControl());

      double desiredLeftMultiplier = 9.81 / (2.0 * (msg.getDesiredComPosition().getZ() - desiredLeftPose.getPosition().getZ()));

      msg.getLeftDesiredFinalControl().setMultiplier(desiredLeftMultiplier);

      double desiredRightMultiplier = 9.81 / (2.0 * (msg.getDesiredComPosition().getZ() - desiredRightPose.getPosition().getZ()));

      msg.getRightDesiredFinalControl().setMultiplier(desiredRightMultiplier);

      return msg;
   }

}
