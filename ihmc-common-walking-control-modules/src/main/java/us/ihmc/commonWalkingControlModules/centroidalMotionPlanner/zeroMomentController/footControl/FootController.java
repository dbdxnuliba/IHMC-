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
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return null;
   }

   public enum FootControlMode
   {
      FREE_MOTION, CONTACT
   }

   private abstract class FootControlState implements State
   {

   }

   private class FreeMotionControlState extends FootControlState
   {
      private final FootControlMode stateEnum = FootControlMode.FREE_MOTION;

      public FreeMotionControlState(YoVariableRegistry registry)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }

      @Override
      public void doAction(double timeInState)
      {

      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void onExit()
      {

      }
   }

   private class ContactControlState extends FootControlState
   {
      private final RhoRampingProfile toeRhoProvider;
      private final RhoRampingProfile heelRhoProvider;
      private final YoBoolean useRamping;
      private final YoDouble rampingDuration;

      private final YoBoolean isToeLoaded;
      private final YoBoolean isToeLoadingRequested;
      private final YoBoolean isToeTransitioning;
      private final YoDouble currentToeRhoWeight;
      private final YoDouble desiredToeRhoWeight;
      private final YoDouble plannedFinalToeRhoWeight;

      private final YoBoolean isHeelLoaded;
      private final YoBoolean isHeelLoadingRequested;
      private final YoBoolean isHeelTransitioning;
      private final YoDouble currentHeelRhoWeight;
      private final YoDouble desiredHeelRhoWeight;
      private final YoDouble plannedFinalHeelRhoWeight;

      private final YoDouble nominalRhoWeight;
      private final YoDouble maxRhoWeight;
      private final YoDouble minRhoWeight;

      private final YoPlaneContactState contactState;

      public ContactControlState(String namePrefix, YoPlaneContactState contactState, YoVariableRegistry registry)
      {
         isToeLoaded = new YoBoolean(namePrefix + "IsToeLoaded", registry);
         isToeLoadingRequested = new YoBoolean(namePrefix + "IsToeLoadingRequested", registry);
         isToeTransitioning = new YoBoolean(namePrefix + "IsToeTransitioning", registry);
         toeRhoProvider = new RhoRampingProfile(namePrefix + "Toe", registry);
         currentToeRhoWeight = new YoDouble(namePrefix + "CurrentToeRhoWeight", registry);
         desiredToeRhoWeight = new YoDouble(namePrefix + "CurrentToeRhoWeight", registry);
         plannedFinalToeRhoWeight = new YoDouble(namePrefix + "CurrentToeRhoWeight", registry);

         isHeelLoaded = new YoBoolean(namePrefix + "IsHeelLoaded", registry);
         isHeelLoadingRequested = new YoBoolean(namePrefix + "IsHeelLoadingRequested", registry);
         isHeelTransitioning = new YoBoolean(namePrefix + "IsHeelTransitioning", registry);
         heelRhoProvider = new RhoRampingProfile(namePrefix + "Heel", registry);
         currentHeelRhoWeight = new YoDouble(namePrefix + "CurrentHeelRhoWeight", registry);
         desiredHeelRhoWeight = new YoDouble(namePrefix + "CurrentToeRhoWeight", registry);
         plannedFinalHeelRhoWeight = new YoDouble(namePrefix + "CurrentToeRhoWeight", registry);

         useRamping = new YoBoolean(footName + "UseRamping", registry);
         rampingDuration = new YoDouble(namePrefix + "RampingDuration", registry);

         nominalRhoWeight = new YoDouble(namePrefix + "NominalRhoWeight", registry);
         maxRhoWeight = new YoDouble(namePrefix + "MaxRhoWeight", registry);
         minRhoWeight = new YoDouble(namePrefix + "MinRhoWeight", registry);

         this.contactState = contactState;
      }

      public void setParameters(double minRhoWeight, double maxRhoWeight, double nominalRhoWeight)
      {
         this.nominalRhoWeight.set(nominalRhoWeight);
         this.maxRhoWeight.set(maxRhoWeight);
         this.minRhoWeight.set(minRhoWeight);
      }

      public void disableRamping()
      {
         useRamping.set(false);
         rampingDuration.setToNaN();
      }

      public void enableRamping(double duration)
      {
         useRamping.set(true);
         rampingDuration.set(duration);
      }

      public void requestToeLoading(boolean shouldToeBeLoaded)
      {
         isToeLoadingRequested.set(shouldToeBeLoaded);
         if (shouldToeBeLoaded)
            desiredToeRhoWeight.set(nominalRhoWeight.getDoubleValue());
         else
            desiredToeRhoWeight.set(maxRhoWeight.getDoubleValue());
      }

      public void requestHeelLoading(boolean shouldHeelBeLoaded)
      {
         isHeelLoadingRequested.set(shouldHeelBeLoaded);
         if (shouldHeelBeLoaded)
            desiredHeelRhoWeight.set(nominalRhoWeight.getDoubleValue());
         else
            desiredHeelRhoWeight.set(maxRhoWeight.getDoubleValue());
      }

      @Override
      public boolean isDone(double timeInState)
      {
         boolean isHeelInRequestedState = false;
         boolean isToeInRequestedState = false;
         return isHeelInRequestedState && isToeInRequestedState;
      }

      @Override
      public void doAction(double timeInState)
      {
         // Create plans if desireds and planned finals don't match
         if (MathTools.epsilonCompare(plannedFinalHeelRhoWeight.getDoubleValue(), desiredHeelRhoWeight.getDoubleValue(), Epsilons.ONE_BILLIONTH))
            createHeelRhoRampPlan(timeInState);
         if (MathTools.epsilonCompare(plannedFinalToeRhoWeight.getDoubleValue(), desiredToeRhoWeight.getDoubleValue(), Epsilons.ONE_BILLIONTH))
            createToeRhoRampPlan(timeInState);

         // Compute rhos for states 
         if (isToeTransitioning.getBooleanValue())
            computeToeRhoWeight(timeInState);
         if (isHeelTransitioning.getBooleanValue())
            computeHeelRhoWeight(timeInState);

         // Decide which contact points to enable
         boolean heelIsUnderActiveControl = false;
         boolean toeIsUnderActiveControl = false;
         setContactPlaneStateToComputedValues(heelIsUnderActiveControl, toeIsUnderActiveControl);
      }

      private void setContactPlaneStateToComputedValues(boolean heelIsUnderActiveControl, boolean toeIsUnderActiveControl)
      {
         int numberOfContactPoints = contactState.getTotalNumberOfContactPoints();
         List<YoContactPoint> contactPointList = contactState.getContactPoints();
         double toeX = contactState.getContactPointLabels().getToeX();
         double heelX = contactState.getContactPointLabels().getHeelX();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            boolean isHeelVertex = contactState.isHeelContactPoint(i);
            boolean isToeVertex = contactState.isToeContactPoint(i);
            YoContactPoint contactPoint = contactPointList.get(i);
            if (isHeelVertex)
            {
               contactState.setContactPointInContact(i, heelIsUnderActiveControl);
               if (heelIsUnderActiveControl)
               {
                  contactState.setMaxContactPointNormalForce(contactPoint, currentHeelRhoWeight.getDoubleValue());
                  //contactState.setRhoWeight(contactPoint, rhoWeight);
               }
            }
            else if (isToeVertex)
            {
               contactState.setContactPointInContact(i, toeIsUnderActiveControl);
               if (toeIsUnderActiveControl)
               {
                  contactState.setMaxContactPointNormalForce(contactPoint, currentToeRhoWeight.getDoubleValue());
                  //contactState.setRhoWeight(contactPoint, rhoWeight);
               }
            }
            else
            {
               boolean computeMidPointWeights = heelIsUnderActiveControl && toeIsUnderActiveControl;
               contactState.setContactPointInContact(i, computeMidPointWeights);
               if (computeMidPointWeights)
               {
                  double vertexX = contactPoint.getPosition().getX();
                  double maxValue = (vertexX - heelX) / (toeX - heelX) * (currentToeRhoWeight.getDoubleValue() - currentHeelRhoWeight.getDoubleValue())
                        + currentHeelRhoWeight.getDoubleValue();
                  contactState.setMaxContactPointNormalForce(contactPoint, maxValue);
                  //contactState.setRhoWeight(contactPoint, rhoWeight);
               }
            }

         }
      }

      private void createToeRhoRampPlan(double timeInState)
      {
         plannedFinalToeRhoWeight.set(desiredToeRhoWeight.getDoubleValue());
         isToeTransitioning.set(true);
         toeRhoProvider.createRhoRamp(timeInState, timeInState + rampingDuration.getDoubleValue(), currentToeRhoWeight.getDoubleValue(),
                                      plannedFinalToeRhoWeight.getDoubleValue());
      }

      private void createHeelRhoRampPlan(double timeInState)
      {
         plannedFinalHeelRhoWeight.set(desiredHeelRhoWeight.getDoubleValue());
         isHeelTransitioning.set(true);
         heelRhoProvider.createRhoRamp(timeInState, timeInState + rampingDuration.getDoubleValue(), currentHeelRhoWeight.getDoubleValue(),
                                       plannedFinalHeelRhoWeight.getDoubleValue());
      }

      private void computeToeRhoWeight(double timeInState)
      {
         currentToeRhoWeight.set(toeRhoProvider.compute(timeInState));
         if (timeInState >= toeRhoProvider.getFinalTime())
            isToeTransitioning.set(false);
      }

      private void computeHeelRhoWeight(double timeInState)
      {
         currentHeelRhoWeight.set(heelRhoProvider.compute(timeInState));
         if (timeInState >= heelRhoProvider.getFinalTime())
            isHeelTransitioning.set(false);
      }

      @Override
      public void onEntry()
      {
         isToeLoaded.set(false);
         isToeTransitioning.set(false);
         isHeelLoaded.set(false);
         isHeelTransitioning.set(false);
      }

      @Override
      public void onExit()
      {
         disableRamping();
         isToeLoaded.set(false);
         isHeelLoaded.set(false);
      }
   }
}
