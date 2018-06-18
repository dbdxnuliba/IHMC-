package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ContactControlState extends FootControlState
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
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private final FrameVector3D linearSpatialAccelerationWeight = new FrameVector3D();
   private final FrameVector3D angularSpatialAccelerationWeight = new FrameVector3D();
   private final FrameVector3D linearFeedbackControlWeight = new FrameVector3D();
   private final FrameVector3D angularFeedbackControlWeight = new FrameVector3D();

   private final RigidBody rootBody;
   private final RigidBody pelvis;
   private final RigidBody foot;
   private final ReferenceFrame soleFrame;
   private final FramePoint3D controlPoint;
   private final FrameQuaternion controlOrientation;
   private final PoseReferenceFrame controlFrame;
   private final ContactableFoot contactableFoot;

   private final List<FramePoint3DReadOnly> toePoints = new ArrayList<>();
   private final List<FramePoint3DReadOnly> heelPoints = new ArrayList<>();
   private final FramePoint3D toeControlPoint = new FramePoint3D();
   private final FramePoint3D heelControlPoint = new FramePoint3D();
   private final FramePoint3D footControlPoint = new FramePoint3D();

   public ContactControlState(String namePrefix, YoPlaneContactState contactState, ContactableFoot contactableFoot, RigidBody rootBody, RigidBody pelvis,
                              YoVariableRegistry registry)
   {
      isToeLoaded = new YoBoolean(namePrefix + "IsToeLoaded", registry);
      isToeLoadingRequested = new YoBoolean(namePrefix + "IsToeLoadingRequested", registry);
      isToeTransitioning = new YoBoolean(namePrefix + "IsToeTransitioning", registry);
      toeRhoProvider = new RhoRampingProfile(namePrefix + "Toe", registry);
      currentToeRhoWeight = new YoDouble(namePrefix + "CurrentToeRhoWeight", registry);
      desiredToeRhoWeight = new YoDouble(namePrefix + "DesiredToeRhoWeight", registry);
      plannedFinalToeRhoWeight = new YoDouble(namePrefix + "PlannedFinalToeRhoWeight", registry);

      isHeelLoaded = new YoBoolean(namePrefix + "IsHeelLoaded", registry);
      isHeelLoadingRequested = new YoBoolean(namePrefix + "IsHeelLoadingRequested", registry);
      isHeelTransitioning = new YoBoolean(namePrefix + "IsHeelTransitioning", registry);
      heelRhoProvider = new RhoRampingProfile(namePrefix + "Heel", registry);
      currentHeelRhoWeight = new YoDouble(namePrefix + "CurrentHeelRhoWeight", registry);
      desiredHeelRhoWeight = new YoDouble(namePrefix + "DesiredHeelRhoWeight", registry);
      plannedFinalHeelRhoWeight = new YoDouble(namePrefix + "PlannedFinalHeelRhoWeight", registry);

      useRamping = new YoBoolean(namePrefix + "UseRamping", registry);
      rampingDuration = new YoDouble(namePrefix + "RampingDuration", registry);

      nominalRhoWeight = new YoDouble(namePrefix + "NominalRhoWeight", registry);
      maxRhoWeight = new YoDouble(namePrefix + "MaxRhoWeight", registry);
      minRhoWeight = new YoDouble(namePrefix + "MinRhoWeight", registry);

      this.contactState = contactState;
      this.contactableFoot = contactableFoot;
      this.soleFrame = contactableFoot.getSoleFrame();
      this.controlOrientation = new FrameQuaternion(soleFrame);
      this.controlPoint = new FramePoint3D(soleFrame);
      this.controlFrame = new PoseReferenceFrame(namePrefix + "ControlFrame", soleFrame);
      this.foot = contactableFoot.getRigidBody();
      this.rootBody = rootBody;
      this.pelvis = pelvis;

      setupControlPoints(contactState);
      setupSpatialAccelerationCommand();
      setupFeedbackControlCommand();
   }

   private void setupControlPoints(YoPlaneContactState contactState)
   {
      FramePoint3D tempPoint = new FramePoint3D();
      int numberOfToePoints = 0;
      int numberOfHeelPoints = 0;
      heelControlPoint.setToZero(soleFrame);
      toeControlPoint.setToZero(soleFrame);
      for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = contactState.getContactPoints().get(i);
         if (contactState.isHeelContactPoint(i))
         {
            tempPoint.setIncludingFrame(contactPoint.getPosition());
            tempPoint.changeFrame(soleFrame);
            heelPoints.add(tempPoint);
            heelControlPoint.add(tempPoint);
            numberOfHeelPoints++;
         }
         if (contactState.isToeContactPoint(i))
         {
            tempPoint.setIncludingFrame(contactPoint.getPosition());
            tempPoint.changeFrame(soleFrame);
            toePoints.add(tempPoint);
            toeControlPoint.add(tempPoint);
            numberOfToePoints++;
         }
      }
      if (numberOfHeelPoints == 0)
         throw new RuntimeException("Contact state does not have any heel contacts");
      if (numberOfToePoints == 0)
         throw new RuntimeException("Contact state does not have any toe contacts");
      heelControlPoint.scale(1.0 / numberOfHeelPoints);
      toeControlPoint.scale(1.0 / numberOfToePoints);
      footControlPoint.setToZero(soleFrame);
      footControlPoint.interpolate(heelControlPoint, toeControlPoint, 0.5);
   }

   private void setupSpatialAccelerationCommand()
   {
      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, foot);
      spatialAccelerationCommand.setPrimaryBase(pelvis);
      spatialAccelerationCommand.setLinearWeights(linearSpatialAccelerationWeight);
      spatialAccelerationCommand.setAngularWeights(angularSpatialAccelerationWeight);
   }

   private void setupFeedbackControlCommand()
   {

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
         desiredToeRhoWeight.set(0.0);
   }

   public void requestHeelLoading(boolean shouldHeelBeLoaded)
   {
      isHeelLoadingRequested.set(shouldHeelBeLoaded);
      if (shouldHeelBeLoaded)
         desiredHeelRhoWeight.set(nominalRhoWeight.getDoubleValue());
      else
         desiredHeelRhoWeight.set(0.0);
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
      if (useRamping.getBooleanValue())
      {
         // Create plans if desireds and planned finals don't match
         if (!MathTools.epsilonCompare(plannedFinalHeelRhoWeight.getDoubleValue(), desiredHeelRhoWeight.getDoubleValue(), Epsilons.ONE_BILLIONTH))
            createHeelRhoRampPlan(timeInState);
         if (!MathTools.epsilonCompare(plannedFinalToeRhoWeight.getDoubleValue(), desiredToeRhoWeight.getDoubleValue(), Epsilons.ONE_BILLIONTH))
            createToeRhoRampPlan(timeInState);

         // Compute rhos for states 
         if (isToeTransitioning.getBooleanValue())
            computeToeRhoWeight(timeInState);
         if (isHeelTransitioning.getBooleanValue())
            computeHeelRhoWeight(timeInState);
      }
      else
      {
         plannedFinalHeelRhoWeight.set(desiredHeelRhoWeight.getDoubleValue());
         currentHeelRhoWeight.set(desiredHeelRhoWeight.getDoubleValue());
         plannedFinalToeRhoWeight.set(desiredToeRhoWeight.getDoubleValue());
         currentToeRhoWeight.set(desiredToeRhoWeight.getDoubleValue());
      }

      isHeelLoaded.set(isHeelTransitioning.getBooleanValue() || isHeelLoadingRequested.getBooleanValue());
      isToeLoaded.set(isToeTransitioning.getBooleanValue() || isToeLoadingRequested.getBooleanValue());
      // Decide which contact points to enable
      boolean heelIsUnderActiveControl = isHeelLoaded.getBooleanValue();
      boolean toeIsUnderActiveControl = isToeLoaded.getBooleanValue();
      setContactPlaneStateToComputedValues(heelIsUnderActiveControl, toeIsUnderActiveControl);
      updateControlPose();
      computeInverseDynamicsCommand();
      //computeSpatialFeedbackCommand();
   }

   private void updateControlPose()
   {
      if (isHeelLoaded.getBooleanValue() && isToeLoaded.getBooleanValue())
         controlPoint.set(footControlPoint);
      else if (isHeelLoaded.getBooleanValue())
         controlPoint.set(heelControlPoint);
      else if (isToeLoaded.getBooleanValue())
         controlPoint.set(toeControlPoint);
      else
         controlPoint.setToNaN();
   }

   private void computeSpatialFeedbackCommand()
   {

   }

   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

   private void computeInverseDynamicsCommand()
   {
      controlFrame.setPoseAndUpdate(controlPoint, controlOrientation);
      desiredSpatialAcceleration.setToZero(foot.getBodyFixedFrame(), rootBody.getBodyFixedFrame(), controlFrame);
      spatialAccelerationCommand.setSpatialAcceleration(controlFrame, desiredSpatialAcceleration);
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
            contactState.setMaxContactPointNormalForce(contactPoint, currentHeelRhoWeight.getDoubleValue());
         }
         else if (isToeVertex)
         {
            contactState.setContactPointInContact(i, toeIsUnderActiveControl);
            contactState.setMaxContactPointNormalForce(contactPoint, currentToeRhoWeight.getDoubleValue());
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
            else
               contactState.setMaxContactPointNormalForce(contactPoint, 0.0);
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

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null; //spatialFeedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      SpatialFeedbackControlCommand spatialFeedbackControlCommandTemplate = new SpatialFeedbackControlCommand();
      return null; //spatialFeedbackControlCommandTemplate;
   }
}
