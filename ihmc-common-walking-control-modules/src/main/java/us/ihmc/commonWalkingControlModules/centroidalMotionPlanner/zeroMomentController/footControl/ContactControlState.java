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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
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

   private final Vector3D defaultLinearSpatialAccelerationWeight = new Vector3D(50.0, 50.0, 50.0);
   private final Vector3D defaultAngularSpatialAccelerationWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Vector3D defaultLinearFeedbackControlWeight = new Vector3D(50.0, 50.0, 50.0);
   private final Vector3D defaultAngularFeedbackControlWeight = new Vector3D(1.0, 1.0, 1.0);

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
   private final FrameVector3D heelControlVector = new FrameVector3D();
   private final FrameVector3D toeControlVector = new FrameVector3D();

   private final SelectionMatrix6D spatialAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackCommandSelectionMatrix = new SelectionMatrix6D();

   private final YoBoolean isDampingActive;
   private final YoBoolean isFeedbackControlActive;

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

      isDampingActive = new YoBoolean(namePrefix + "IsDampingActive", registry);
      isFeedbackControlActive = new YoBoolean(namePrefix + "IsFeedbackControlActive", registry);

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
      footControlPoint.setToZero(soleFrame);
      for (int i = 0; i < contactState.getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = contactState.getContactPoints().get(i);
         tempPoint.setIncludingFrame(contactPoint.getPosition());
         tempPoint.changeFrame(soleFrame);
         if (contactState.isHeelContactPoint(i))
         {
            heelPoints.add(tempPoint);
            heelControlPoint.add(tempPoint);
            numberOfHeelPoints++;
         }
         if (contactState.isToeContactPoint(i))
         {
            toePoints.add(tempPoint);
            toeControlPoint.add(tempPoint);
            numberOfToePoints++;
         }
         footControlPoint.add(tempPoint);
      }
      if (numberOfHeelPoints == 0)
         throw new RuntimeException("Contact state does not have any heel contacts");
      if (numberOfToePoints == 0)
         throw new RuntimeException("Contact state does not have any toe contacts");
      heelControlPoint.scale(1.0 / numberOfHeelPoints);
      toeControlPoint.scale(1.0 / numberOfToePoints);
      footControlPoint.scale(1.0 / contactState.getTotalNumberOfContactPoints());
      if (numberOfHeelPoints == 1)
         heelControlVector.setToNaN();
      else
      {
         heelControlVector.sub(heelPoints.get(0), heelPoints.get(1));
         heelControlVector.normalize();
      }

      if (numberOfToePoints == 1)
         toeControlVector.setToNaN();
      else
      {
         toeControlVector.sub(toePoints.get(0), toePoints.get(1));
         toeControlVector.normalize();
      }
   }

   private void setupSpatialAccelerationCommand()
   {
      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, foot);
      spatialAccelerationCommand.setPrimaryBase(pelvis);
   }

   private void setupFeedbackControlCommand()
   {
      spatialFeedbackControlCommand.set(rootBody, foot);
      spatialAccelerationCommand.setPrimaryBase(pelvis);
      spatialFeedbackControlCommand.setLinearWeightsForSolver(defaultLinearFeedbackControlWeight);
      spatialFeedbackControlCommand.setAngularWeightsForSolver(defaultAngularFeedbackControlWeight);
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
      boolean isHeelFree = !isHeelLoaded.getBooleanValue();
      boolean isToeFree = !isToeLoaded.getBooleanValue();
      return isHeelFree && isToeFree;
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
      updateControlPoseAndUpdateSelectionMatrix();
      computeControllerCoreCommand();
   }

   private void updateControlPoseAndUpdateSelectionMatrix()
   {
      spatialAccelerationSelectionMatrix.setSelectionFrame(controlFrame);
      feedbackCommandSelectionMatrix.setSelectionFrame(controlFrame);
      if (isHeelLoaded.getBooleanValue() && isToeLoaded.getBooleanValue())
      {
         isDampingActive.set(true);
         isFeedbackControlActive.set(false);
         spatialAccelerationSelectionMatrix.resetSelection();
         feedbackCommandSelectionMatrix.clearSelection();

         controlPoint.set(footControlPoint);
         controlOrientation.setToZero(soleFrame);
      }
      else if (isHeelLoaded.getBooleanValue() || isToeLoaded.getBooleanValue())
      {
         isDampingActive.set(true);
         isFeedbackControlActive.set(true);
         List<FramePoint3DReadOnly> contactPoints = isHeelLoaded.getBooleanValue() ? heelPoints : toePoints;
         FramePoint3DReadOnly desiredControlPoint = isHeelLoaded.getBooleanValue() ? heelControlPoint : toeControlPoint;
         if (contactPoints.size() > 1)
         {
            spatialAccelerationSelectionMatrix.resetLinearSelection();
            spatialAccelerationSelectionMatrix.selectAngularX(true);
            spatialAccelerationSelectionMatrix.selectAngularZ(true);
            feedbackCommandSelectionMatrix.clearSelection();
            feedbackCommandSelectionMatrix.selectAngularY(true);
            double yaw = 0.0;
            controlPoint.set(desiredControlPoint);
            controlOrientation.setYawPitchRollIncludingFrame(soleFrame, yaw, 0.0, 0.0);
         }
         else
         {
            spatialAccelerationSelectionMatrix.setToLinearSelectionOnly();
            feedbackCommandSelectionMatrix.setToAngularSelectionOnly();
            controlPoint.set(desiredControlPoint);
            controlOrientation.setToZero(soleFrame);
         }
      }
      else
      {
         isDampingActive.set(false);
         isFeedbackControlActive.set(true);
         spatialAccelerationSelectionMatrix.clearSelection();
         feedbackCommandSelectionMatrix.resetSelection();
         // This doesn't matter since all the control is through the feedback command
         controlPoint.set(footControlPoint);
         controlOrientation.setToZero(soleFrame);
      }
   }

   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

   private void computeControllerCoreCommand()
   {
      // Set spatial feedback command
      // Set spatial acceleration command
      controlFrame.setPoseAndUpdate(controlPoint, controlOrientation);
      desiredSpatialAcceleration.setToZero(foot.getBodyFixedFrame(), rootBody.getBodyFixedFrame(), controlFrame);
      spatialAccelerationCommand.setSpatialAcceleration(controlFrame, desiredSpatialAcceleration);
      spatialAccelerationCommand.setSelectionMatrix(spatialAccelerationSelectionMatrix);
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
      return isDampingActive.getBooleanValue() ? spatialAccelerationCommand : null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return isFeedbackControlActive.getBooleanValue() ? spatialFeedbackControlCommand : null;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      SpatialFeedbackControlCommand spatialFeedbackControlCommandTemplate = new SpatialFeedbackControlCommand();
      return null; //spatialFeedbackControlCommandTemplate;
   }
}
