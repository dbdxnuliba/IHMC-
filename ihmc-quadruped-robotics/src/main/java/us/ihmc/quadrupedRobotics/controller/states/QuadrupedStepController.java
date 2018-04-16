package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSpaceManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStep;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedStepController implements QuadrupedController, QuadrupedStepTransitionCallback
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStepMessageHandler stepMessageHandler;

   // managers
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedJointSpaceManager jointSpaceManager;

   // task space controller
   private final QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();

   // step planner
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint3D> groundPlanePositions;

   // inputs
   private final YoBoolean onLiftOffTriggered = new YoBoolean("onLiftOffTriggered", registry);
   private final YoBoolean onTouchDownTriggered = new YoBoolean("onTouchDownTriggered", registry);

   private final QuadrupedControllerToolbox controllerToolbox;

   public QuadrupedStepController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                  QuadrupedStepMessageHandler stepMessageHandler, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.stepMessageHandler = stepMessageHandler;

      // feedback controllers
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      // step planner
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new YoFramePoint3D(robotQuadrant.getCamelCaseName() + "GroundPlanePosition", worldFrame, registry));
         contactStates.put(robotQuadrant, ContactState.IN_CONTACT);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void onLiftOff(RobotQuadrant thisStepQuadrant)
   {
      // update ground plane estimate
      groundPlanePositions.get(thisStepQuadrant).setMatchingFrame(controllerToolbox.getTaskSpaceEstimates().getSolePosition(thisStepQuadrant));
      onLiftOffTriggered.set(true);
   }

   @Override
   public void onTouchDown(RobotQuadrant thisStepQuadrant)
   {
      onTouchDownTriggered.set(true);
   }

   @Override
   public void onEntry()
   {
      // initialize state
      stepMessageHandler.initialize();
      onLiftOffTriggered.set(false);
      onTouchDownTriggered.set(false);

      // update task space estimates
      controllerToolbox.update();
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();

      bodyOrientationManager.initialize(taskSpaceEstimates.getBodyOrientation());
      bodyOrientationManager.setDesiredFrameToHoldPosition(controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());

      feetManager.registerStepTransitionCallback(this);
      feetManager.reset();
      feetManager.requestFullContact();

      // initialize task space controller
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactStates.put(robotQuadrant, ContactState.IN_CONTACT);
      }

      // initialize ground plane
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.get(robotQuadrant).setMatchingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();

      stepMessageHandler.process(balanceManager.getAccumulatedStepAdjustment());
      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(stepMessageHandler.getStepSequence());

      // compute step adjustment
      RecyclingArrayList<QuadrupedStep> adjustedSteps = balanceManager.computeStepAdjustment(stepMessageHandler.getActiveSteps());
      feetManager.adjustSteps(adjustedSteps);

      balanceManager.initializeForStepping(contactStates);
   }

   @Override
   public void doAction(double timeInState)
   {
      stepMessageHandler.process(balanceManager.getAccumulatedStepAdjustment());

      // update task space estimates
      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      // trigger step events
      feetManager.triggerSteps(stepMessageHandler.getActiveSteps());

      // update ground plane estimate
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();

      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(stepMessageHandler.getStepSequence());

      // update step adjustment
      RecyclingArrayList<QuadrupedStep> adjustedSteps = balanceManager.computeStepAdjustment(stepMessageHandler.getActiveSteps());
      feetManager.adjustSteps(adjustedSteps);

      // update desired horizontal com forces
      balanceManager.compute(contactStates);

      // update desired body orientation, angular velocity, and torque
      bodyOrientationManager.compute();

      // update desired contact state and sole forces
      feetManager.compute();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactStates.put(robotQuadrant, feetManager.getContactState(robotQuadrant));
      }

      jointSpaceManager.compute();

      // update accumulated step adjustment
      if (onLiftOffTriggered.getBooleanValue())
      {
         onLiftOffTriggered.set(false);
      }
      if (onTouchDownTriggered.getBooleanValue())
      {
         onTouchDownTriggered.set(false);
         balanceManager.completedStep();
      }
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      if (stepMessageHandler.isDoneWithStepSequence())
      {
         return ControllerEvent.DONE;
      }
      return null;
   }

   @Override
   public void onExit()
   {
      stepMessageHandler.reset();
      feetManager.registerStepTransitionCallback(null);
   }

   public void halt()
   {
      stepMessageHandler.halt();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}