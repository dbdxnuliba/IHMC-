package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.screwTheory.RigidBody;
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
   private final String namePrefix;
   private final ContactableFoot contactableFoot;
   private final RigidBody elevator;
   private final RigidBody pelvis;
   private ContactControlState contactState;
   private FreeMotionControlState freeMotionState;
   
   public FootController(String namePrefix, YoDouble yoTime, YoPlaneContactState footContactState, ContactableFoot contactableFoot, RigidBody elevator,
                         RigidBody pelvis, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      registry = new YoVariableRegistry(namePrefix + "Controller");
      this.soleFrame = contactableFoot.getSoleFrame();
      this.footContactState = footContactState;
      this.pelvis = pelvis;
      this.elevator = elevator;
      this.contactableFoot = contactableFoot;
      requestedFootState = new YoEnum<>(namePrefix + "RequestedState", registry, FootControlMode.class, true);
      stateMachine = setupStateMachine(yoTime);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      requestedFootState.set(FootControlMode.CONTACT);
      stateMachine.resetToInitialState();
      contactState.requestHeelLoading(true);
      contactState.requestToeLoading(true);
      contactState.disableRamping();
   }

   public void setParameters(double minRhoWeight, double maxRhoWeight, double nominalRho)
   {
      contactState.setParameters(minRhoWeight, maxRhoWeight, nominalRho);
   }

   public void requestTransitionToFreeMotion()
   {
      requestedFootState.set(FootControlMode.FREE_MOTION);
      contactState.disableRamping();
      contactState.requestToeLoading(false);
      contactState.requestHeelLoading(false);
   }

   public void requestTransitionToFreeMotion(double rampingDuration)
   {
      requestedFootState.set(FootControlMode.FREE_MOTION);
      contactState.enableRamping(rampingDuration);
      contactState.requestToeLoading(false);
      contactState.requestHeelLoading(false);
   }

   public void requestTransitionToContact(boolean loadToe, boolean loadHeel)
   {
      requestedFootState.set(FootControlMode.CONTACT);
      contactState.disableRamping();
      contactState.requestToeLoading(loadToe);
      contactState.requestHeelLoading(loadHeel);
   }

   public void requestTransitionToContact(double rampingDuration, boolean loadToe, boolean loadHeel)
   {
      requestedFootState.set(FootControlMode.CONTACT);
      contactState.enableRamping(rampingDuration);
      contactState.requestToeLoading(loadToe);
      contactState.requestHeelLoading(loadHeel);
   }

   private StateMachine<FootControlMode, FootControlState> setupStateMachine(YoDouble yoTime)
   {
      StateMachineFactory<FootControlMode, FootControlState> factory = new StateMachineFactory<>(FootControlMode.class);
      factory.setNamePrefix(namePrefix + "Controller").setRegistry(registry).buildYoClock(yoTime);
      freeMotionState = new FreeMotionControlState(registry);
      factory.addState(FootControlMode.FREE_MOTION, freeMotionState);
      contactState = new ContactControlState(namePrefix, footContactState, contactableFoot, elevator, pelvis, registry);
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
