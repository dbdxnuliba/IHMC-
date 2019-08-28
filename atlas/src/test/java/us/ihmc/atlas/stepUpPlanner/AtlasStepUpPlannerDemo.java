package us.ihmc.atlas.stepUpPlanner;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.stepUpPlanner.StepUpPlannerRequester;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.Ros2Node;

public class AtlasStepUpPlannerDemo
{
   private final HumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJointsExcludingHands;
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationData = new AtomicReference<>(null);

   private boolean updateFullRobotModel()
   {
      RobotConfigurationData robotData = latestRobotConfigurationData.get();

      if (robotData == null)
      {
         LogTools.error("No robot configuration data!");
         return false;
      }

      for (int i = 0; i < robotData.getJointAngles().size(); i++)
         allJointsExcludingHands[i].setQ(robotData.getJointAngles().get(i));

      fullRobotModel.getRootJoint().getJointPose().setPosition(robotData.getRootTranslation());
      fullRobotModel.getRootJoint().getJointPose().setOrientation(robotData.getRootOrientation());

      referenceFrames.updateFrames();

      return true;
   }

   private void waitUpdateFullRobotModel()
   {
      while (!updateFullRobotModel())
      {
         ThreadTools.sleep(500);
      }
   }

   private AtlasStepUpPlannerDemo(String robotName, FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stepUpPlanner_AtlasDemo");

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> latestRobotConfigurationData.set(s.takeNextData()));
      ThreadTools.sleep(1000);

   }

   public static void main(String[] args)
   {
      /*----------------- Hard coded parameters --------*/
      // Other hard coded parameters are in the methods StepUpPlannerRequester.getDefaultFivePhasesParametersMessage and 
      // StepUpPlannerRequester.getDefaultFivePhasesRequestMessage

      double maxLegLength = 1.15;
      double desiredLegLength = 1.05;
      double stepHeight = 0.154*2;
      double stepLength = 0.55;
      double footScale = 0.3;
      /*------------------------------------------------*/

      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      AtlasStepUpPlannerDemo demo = new AtlasStepUpPlannerDemo(atlasRobotModel.getSimpleRobotName(), atlasRobotModel.createFullRobotModel());

      demo.waitUpdateFullRobotModel();

      ReferenceFrame initialCoMFrame = demo.referenceFrames.getCenterOfMassFrame();
      FramePose3D comPose = new FramePose3D(initialCoMFrame);
      comPose.changeFrame(ReferenceFrame.getWorldFrame());

      MovingReferenceFrame pelvisZUpFrame = demo.referenceFrames.getPelvisZUpFrame();
      FramePose3D pelvisFrame = new FramePose3D(pelvisZUpFrame);
      pelvisFrame.changeFrame(ReferenceFrame.getWorldFrame());
      double heightDifference = pelvisFrame.getPosition().getZ() - comPose.getPosition().getZ();

      StepUpPlannerRespondMessage receivedRespond;
      StepUpPlannerRequester requester = new StepUpPlannerRequester();
      StepUpPlannerParametersMessage parameters = StepUpPlannerRequester.getDefaultFivePhasesParametersMessage(atlasRobotModel.getWalkingControllerParameters()
                                                                                                                              .getSteppingParameters(),
                                                                                                               heightDifference, maxLegLength, footScale);

      parameters.setSendComMessages(true);
      parameters.setIncludeComMessages(false);
      parameters.setComMessagesTopic(ControllerAPIDefinition.getSubscriberTopicNameGenerator(atlasRobotModel.getSimpleRobotName())
                                                            .generateTopicName(CenterOfMassTrajectoryMessage.class));

      parameters.setSendFootstepMessages(true);
      parameters.setIncludeFootstepMessages(false);
      parameters.setFootstepMessagesTopic(ControllerAPIDefinition.getSubscriberTopicNameGenerator(atlasRobotModel.getSimpleRobotName())
                                                                 .generateTopicName(FootstepDataListMessage.class));

      parameters.setSendPelvisHeightMessages(false);
      parameters.setIncludePelvisHeightMessages(false);
      parameters.setPelvisHeightMessagesTopic(ControllerAPIDefinition.getSubscriberTopicNameGenerator(atlasRobotModel.getSimpleRobotName())
                                                                     .generateTopicName(PelvisHeightTrajectoryMessage.class));

      LogTools.info("Sending parameters.");
      boolean ok = requester.publishParametersAndWaitAck(parameters);
      assertTrue(ok);

      ok = demo.updateFullRobotModel();
      assertTrue(ok);

      StepUpPlannerRequestMessage request = StepUpPlannerRequester.getDefaultFivePhasesRequestMessage(new Vector3D(stepLength, 0.0, stepHeight),
                                                                                                      desiredLegLength,
                                                                                                      demo.referenceFrames);
      LogTools.info("Sending request.");
      receivedRespond = requester.getRespond(request);
      assertTrue(receivedRespond != null);

   }

}
