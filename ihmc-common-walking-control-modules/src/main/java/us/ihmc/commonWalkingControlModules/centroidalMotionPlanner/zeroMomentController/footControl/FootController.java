package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootController implements ControlManagerInterface
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame soleFrame;
   private final StateMachine<FootControlMode, FootControlState> stateMachine;
   private final YoEnum<FootControlMode> requestedFootState;
   private final YoPlaneContactState footContactState;
   private final String footName;
   private ContactControlState contactState;
   private FreeMotionControlState freeMotionState;

   public FootController(YoDouble yoTime, YoPlaneContactState footContactState, ReferenceFrame soleFrame, RobotSide side, YoVariableRegistry parentRegistry)
   {
      footName = side.getCamelCaseNameForStartOfExpression() + "Foot";
      registry = new YoVariableRegistry(side.getCamelCaseNameForMiddleOfExpression() + "FootController");
      this.soleFrame = soleFrame;
      this.footContactState = footContactState;
      requestedFootState = new YoEnum<>(footName + "RequestedState", registry, FootControlMode.class, true);
      stateMachine = setupStateMachine(yoTime);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      stateMachine.resetToInitialState();
   }

   public void setParameters(double minRhoWeight, double maxRhoWeight, double nominalRho)
   {
      contactState.setParameters(minRhoWeight, maxRhoWeight, nominalRho);
   }

   public void requestTransitionToFreeMotion()
   {

   }

   public void requestTransitionToFreeMotion(double rampingDuration)
   {

   }

   public void requestTransitionToContact(boolean loadToe, boolean loadHeel)
   {
      requestedFootState.set(FootControlMode.CONTACT);
      contactState.disableRamping();
      contactState.requestHeelLoading(loadHeel);
      contactState.requestToeLoading(loadToe);
   }

   public void requestTransitionToContact(double rampingDuration, boolean loadToe, boolean loadHeel)
   {
      requestedFootState.set(FootControlMode.CONTACT);
      contactState.enableRamping(rampingDuration);
      contactState.requestHeelLoading(loadHeel);
      contactState.requestToeLoading(loadToe);
   }

   private StateMachine<FootControlMode, FootControlState> setupStateMachine(YoDouble yoTime)
   {
      StateMachineFactory<FootControlMode, FootControlState> factory = new StateMachineFactory<>(FootControlMode.class);
      factory.setNamePrefix(footName + "Controller").setRegistry(registry).buildYoClock(yoTime);
      freeMotionState = new FreeMotionControlState(registry);
      factory.addState(FootControlMode.FREE_MOTION, freeMotionState);
      contactState = new ContactControlState(footName, footContactState, registry);
      factory.addState(FootControlMode.CONTACT, contactState);

      factory.addRequestedTransition(FootControlMode.FREE_MOTION, FootControlMode.CONTACT, requestedFootState, false);
      factory.addRequestedTransition(FootControlMode.CONTACT, FootControlMode.FREE_MOTION, requestedFootState, true);
      return factory.build(FootControlMode.CONTACT);
   }

   public void doControl()
   {
      stateMachine.doActionAndTransition();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return stateMachine.getCurrentState().createFeedbackControlTemplate();
   }
}
