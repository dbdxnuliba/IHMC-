package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootController implements ControlManagerInterface
{
   private final YoVariableRegistry registry;
   private final FootSupportPolygon supportPolygon;
   private final ReferenceFrame soleFrame;
   private final GenericStateMachine<FootControlMode, FootControlState> stateMachine;
   private final YoEnum<FootControlMode> requestedState;
   private final YoPlaneContactState footContactState;
   private final String footName;
   private ContactControlState contactState;
   private FreeMotionControlState freeMotionState;

   public FootController(YoDouble yoTime, YoPlaneContactState footContactState, ReferenceFrame soleFrame, FootSupportPolygon supportPolygon, RobotSide side,
                         YoVariableRegistry parentRegistry)
   {
      footName = side.getCamelCaseNameForStartOfExpression() + "Foot";
      registry = new YoVariableRegistry(side.getCamelCaseNameForMiddleOfExpression() + "FootController");
      this.supportPolygon = supportPolygon;
      this.soleFrame = soleFrame;
      this.footContactState = footContactState;
      stateMachine = new GenericStateMachine<>(footName + "ControlState", footName + "SwitchTime", FootControlMode.class, yoTime, registry);
      requestedState = new YoEnum<>(footName + "RequestedState", registry, FootControlMode.class);
      setupStateMachine();
      parentRegistry.addChild(registry);
   }

   public void intialize(double minRhoWeight, double maxRhoWeight, double nominalRho)
   {
      contactState.initialize(minRhoWeight, maxRhoWeight, nominalRho);
   }

   public void requestTransitionToFreeMotion()
   {

   }

   public void requestTransitionToFreeMotion(double rampingDuration)
   {
   }

   public void requestTransitionToContact(boolean loadToe, boolean loadHeel)
   {

   }

   public void requestTransitionToContact(double rampingDuration, boolean loadToe, boolean loadHeel)
   {
      contactState.requestHeelLoading(loadHeel);
      contactState.requestToeLoading(loadToe);
   }

   private void setupStateMachine()
   {
      freeMotionState = new FreeMotionControlState(registry);
      freeMotionState.addStateTransition(FootControlMode.CONTACT, new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return requestedState.getEnumValue() == FootControlMode.CONTACT;
         }
      });
      stateMachine.addState(freeMotionState);
      contactState = new ContactControlState(footName, footContactState, supportPolygon, registry);
      contactState.addStateTransition(FootControlMode.FREE_MOTION, new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean hasFreeMotionBeenRequested = requestedState.getEnumValue() == FootControlMode.FREE_MOTION;
            boolean isContactStateDone = contactState.isDone();
            return hasFreeMotionBeenRequested && isContactStateDone;
         }
      });
      stateMachine.addState(contactState);
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public enum FootControlMode
   {
      FREE_MOTION, CONTACT
   }

   private abstract class FootControlState extends FinishableState<FootControlMode>
   {

      public FootControlState(FootControlMode stateEnum)
      {
         super(stateEnum);
      }
   }

   private class FreeMotionControlState extends FootControlState
   {
      private final FootControlMode stateEnum = FootControlMode.FREE_MOTION;

      public FreeMotionControlState(YoVariableRegistry registry)
      {
         super(FootControlMode.FREE_MOTION);
      }

      @Override
      public boolean isDone()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public void doAction()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionIntoAction()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionOutOfAction()
      {
         // TODO Auto-generated method stub

      }
   }

   private class ContactControlState extends FootControlState
   {
      private final FootControlMode stateEnum = FootControlMode.CONTACT;
      private final YoBoolean isToeLoaded;
      private final YoBoolean isHeelLoaded;
      private final YoBoolean requestToeLoading;
      private final YoBoolean requestHeelLoading;
      private final RhoRampingProfile toeRhoProvider;
      private final RhoRampingProfile heelRhoProvider;
      private final YoBoolean useRamping;
      private final YoDouble rampingDuration;
      private final YoDouble currentRhoWeight;
      private final YoDouble nominalRhoWeight;
      private final YoDouble maxRhoWeight;
      private final YoDouble minRhoWeight;

      public ContactControlState(String namePrefix, YoPlaneContactState contactState, FootSupportPolygon footSupportPolygon, YoVariableRegistry registry)
      {
         super(FootControlMode.CONTACT);
         isHeelLoaded = new YoBoolean(namePrefix + "IsHeelLoaded", registry);
         isToeLoaded = new YoBoolean(namePrefix + "IsToeLoaded", registry);
         requestHeelLoading = new YoBoolean(namePrefix + "HeelLoadingRequested", registry);
         requestToeLoading = new YoBoolean(namePrefix + "ToeLoadingRequested", registry);
         useRamping = new YoBoolean(footName + "UseRamping", registry);
         rampingDuration = new YoDouble(namePrefix + "RampingDuration", registry);
         currentRhoWeight = new YoDouble(namePrefix + "CurrentRhoWeight", registry);
         heelRhoProvider = new RhoRampingProfile(namePrefix + "Heel", registry);
         toeRhoProvider = new RhoRampingProfile(namePrefix + "Toe", registry);
         nominalRhoWeight = new YoDouble(namePrefix + "NominalRhoWeight", registry);
         maxRhoWeight = new YoDouble(namePrefix + "MaxRhoWeight", registry);
         minRhoWeight = new YoDouble(namePrefix + "MinRhoWeight", registry);
      }

      public void initialize(double minRhoWeight, double maxRhoWeight, double nominalRhoWeight)
      {
         this.nominalRhoWeight.set(nominalRhoWeight);
         this.maxRhoWeight.set(maxRhoWeight);
         this.minRhoWeight.set(minRhoWeight);
      }

      public void reset()
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
         requestToeLoading.set(shouldToeBeLoaded);
      }

      public void requestHeelLoading(boolean shouldHeelBeLoaded)
      {
         requestHeelLoading.set(shouldHeelBeLoaded);
      }

      public void updatePlan()
      {

      }

      @Override
      public boolean isDone()
      {
         boolean isHeelInRequestedState = requestToeLoading.getBooleanValue() == isToeLoaded.getBooleanValue();
         boolean isToeInRequestedState = requestHeelLoading.getBooleanValue() == isHeelLoaded.getBooleanValue();
         return isHeelInRequestedState && isToeInRequestedState;
      }

      @Override
      public void doAction()
      {
         boolean hasToeStateTransitioned = requestToeLoading.getBooleanValue() != isToeLoaded.getBooleanValue();
         boolean hasHeelStateTransitioned = requestHeelLoading.getBooleanValue() != isHeelLoaded.getBooleanValue();
         if(useRamping.getBooleanValue())
         {
            
         }
         else
         {
            
         }
      }

      @Override
      public void doTransitionIntoAction()
      {

      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }
}
