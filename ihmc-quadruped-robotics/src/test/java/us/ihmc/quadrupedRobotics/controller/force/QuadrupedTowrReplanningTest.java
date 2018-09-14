package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.*;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.planning.trajectoryConverter.TowrReplanningHandler;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public abstract class QuadrupedTowrReplanningTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract QuadrupedInitialPositionParameters getInitialPositionParameters();

   @Before
   public void setup() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      //quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.STEPPED);
      quadrupedTestFactory.setUseNetworking(true);
      //quadrupedTestFactory.setInitialPosition(getInitialPositionParameters());
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 74.7)
   @Test(timeout = 370000)
   public void testQuadrupedTowrTrajectoryReplanningToTarget() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      PrintTools.info("Center of Mass and feet trajectory test to target");
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      // let's have the robot go to a narrower step position

      double stanceWidth = 0.5;
      double stanceLength = 0.9;

      Point3D frontLeftPosition = new Point3D(stanceLength / 2.0, stanceWidth / 2.0, 0.0);
      Point3D frontRightPosition = new Point3D(stanceLength / 2.0, -stanceWidth / 2.0, 0.0);
      Point3D backLeftPosition = new Point3D(-stanceLength / 2.0, stanceWidth / 2.0, 0.0);
      Point3D backRightPosition = new Point3D(-stanceLength / 2.0, -stanceWidth / 2.0, 0.0);

      QuadrupedTimedStepListMessage footsteps = new QuadrupedTimedStepListMessage();
      footsteps.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, frontLeftPosition, 0.1, 0.0, 0.4));
      footsteps.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, backRightPosition, 0.1, 0.3, 0.7));
      footsteps.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, frontRightPosition, 0.1, 0.6, 1.0));
      footsteps.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, backLeftPosition, 0.1, 0.9, 1.3));
      footsteps.setIsExpressedInAbsoluteTime(false);

      stepTeleopManager.pulishTimedStepListToController(footsteps);

      YoBoolean shiftPlanBasedOnStepAdjustment = (YoBoolean) conductor.getScs().getVariable("shiftPlanBasedOnStepAdjustment");
      YoBoolean useStepAdjustment = (YoBoolean) conductor.getScs().getVariable("useStepAdjustment");
      YoBoolean flSpeedUp = (YoBoolean) conductor.getScs().getVariable("FrontLeftIsSwingSpeedUpEnabled");
      YoBoolean frSpeedUp = (YoBoolean) conductor.getScs().getVariable("FrontRightIsSwingSpeedUpEnabled");
      YoBoolean hlSpeedUp = (YoBoolean) conductor.getScs().getVariable("HindLeftIsSwingSpeedUpEnabled");
      YoBoolean hrSpeedUp = (YoBoolean) conductor.getScs().getVariable("HindRightIsSwingSpeedUpEnabled");
      shiftPlanBasedOnStepAdjustment.set(false);
      useStepAdjustment.set(false);
      flSpeedUp.set(false);
      frSpeedUp.set(false);
      hlSpeedUp.set(false);
      hrSpeedUp.set(false);

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      conductor.simulate();


      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "scripted_flat_ground_walking");
      String robotName = quadrupedTestFactory.getRobotName();
      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      AtomicReference<QuadrupedControllerEnum> controllerState = new AtomicReference<>();
      AtomicReference<QuadrupedSteppingStateEnum> steppingState = new AtomicReference<>();
      ros2Node.createCallbackSubscription(QuadrupedControllerStateChangeMessage.getPubSubType(), );
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedControllerStateChangeMessage.class, controllerPubGenerator,
                                           s -> controllerState.set(QuadrupedControllerEnum.fromByte(s.takeNextData().getEndQuadrupedControllerEnum())));

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> steppingState.set(QuadrupedSteppingStateEnum.fromByte(s.takeNextData().getEndQuadrupedSteppingStateEnum())));

      ros2Node.spin();

      QuadrupedRequestedControllerStateMessage controllerMessage = new QuadrupedRequestedControllerStateMessage();
      controllerMessage.setQuadrupedControllerRequestedEvent(QuadrupedControllerRequestedEvent.REQUEST_STEPPING.toByte());
      IHMCROS2Publisher<QuadrupedRequestedControllerStateMessage> controllerStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedControllerStateMessage.class, controllerSubGenerator);
      controllerStatePublisher.publish(controllerMessage);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      boolean isTowrTrajectoryReceived = listenToTowr();

      QuadrupedTimedStepListMessage stepsMessage = getSteps();
      IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);
      timedStepPublisher.publish(stepsMessage);

      CenterOfMassTrajectoryMessage comMessage = getCenterOfMassTrajectoryMessage();
      IHMCROS2Publisher<CenterOfMassTrajectoryMessage> centerOfMassTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, CenterOfMassTrajectoryMessage.class, controllerSubGenerator);
      centerOfMassTrajectoryPublisher.publish(comMessage);

      QuadrupedBodyHeightMessage bodyHeightMessage = getBodyHeightMessage();
      IHMCROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);
      bodyHeightTrajectoryPublisher.publish((bodyHeightMessage));

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 3.0));
      conductor.simulate();

      if(isTowrTrajectoryReceived)
      {
         PrintTools.info("publishing new initial position");
         TowrReplanningHandler towrReplanningHandler = new TowrReplanningHandler();
         State6d newState = new State6d();
         newState.getPose().getPosition().setX(0.3);
         towrReplanningHandler.publishInitialState6D(node, newState);
         isTowrTrajectoryReceived = false;
      }


      PrintTools.info("publishing new initial position");
      TowrReplanningHandler towrReplanningHandler = new TowrReplanningHandler();
      State6d newState = new State6d();
      newState.getPose().getPosition().setX(0.3);
      towrReplanningHandler.publishInitialState6D(newState);

      // check robot is still upright and walked forward
      Point3D expectedFinalPlanarPosition = getFinalPlanarPosition();
      assertEquals(variables.getRobotBodyX().getDoubleValue(), expectedFinalPlanarPosition.getX(), 0.1);
      assertEquals(variables.getRobotBodyY().getDoubleValue(), expectedFinalPlanarPosition.getY(), 0.1);
      assertEquals(variables.getRobotBodyYaw().getDoubleValue(), expectedFinalPlanarPosition.getZ(), 0.1);
      assertTrue(variables.getRobotBodyZ().getDoubleValue() > 0.0);
      conductor.concludeTesting();

      ros2Node.destroy();
   }

   /**
    * Listen to TOWR messages
    * */
   public abstract boolean listenToTowr();

   /**
    * Steps to execute, not expressed in absolute time
    */
   public abstract QuadrupedTimedStepListMessage getSteps();

   /**
    * Steps to execute, not expressed in absolute time
    */
   public abstract CenterOfMassTrajectoryMessage getCenterOfMassTrajectoryMessage();

   /**
    * Body height message to execute
    */
   public abstract QuadrupedBodyHeightMessage getBodyHeightMessage();

   /**
    * Expected final planar position, given as x, y, yaw
    */
   public abstract Point3D getFinalPlanarPosition();

}
