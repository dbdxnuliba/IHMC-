package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.taskExecutor.StateExecutor;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class AvatarKinematicsSimulationController
{
   private final double gravityZ = 9.81;
   private final double integrationDT;
   private final YoDouble yoTime;

   private final FloatingJointBasics rootJoint;
   private final FullHumanoidRobotModel fullRobotModel;
   private final YoVariableRegistry registry;
   private final IHMCROS2Publisher<WalkingStatusMessage> walkingStatusPublisher;
   private final IHMCROS2Publisher<FootstepStatusMessage> footstepStatusPublisher;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<SettableFootSwitch> footSwitches = new SideDependentList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingHighLevelHumanoidController walkingController;

   private final CommandInputManager kinematicsSimulationInputManager;

   private final CommandInputManager walkingInputManager = new CommandInputManager("walking_preview_internal",
                                                                                   ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager walkingOutputManager = new StatusMessageOutputManager(
                                                                                   ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private final StateExecutor taskExecutor = new StateExecutor(StateMachineClock.dummyClock()); // should be dummy?
   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

   public AvatarKinematicsSimulationController(DRCRobotModel robotModel,
                                               Pose3DReadOnly initialPelvisPose,
                                               List<Double> initialJointAngles,
                                               double integrationDT,
                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                               YoVariableRegistry registry,
                                               IHMCROS2Publisher<WalkingStatusMessage> walkingStatusPublisher,
                                               IHMCROS2Publisher<FootstepStatusMessage> footstepStatusPublisher)
   {
      this.integrationDT = integrationDT;
      this.registry = registry;
      this.walkingStatusPublisher = walkingStatusPublisher;
      this.footstepStatusPublisher = footstepStatusPublisher;

      kinematicsSimulationInputManager = new CommandInputManager("ik_simulation", ControllerAPIDefinition.getControllerSupportedCommands());
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      yoTime = new YoDouble("timeInPreview", registry);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      // Create registries to match controller so the XML gets loaded properly.
      YoVariableRegistry drcControllerThreadRegistry = new YoVariableRegistry("DRCControllerThread");
      YoVariableRegistry drcMomentumBasedControllerRegistry = new YoVariableRegistry("DRCMomentumBasedController");
      YoVariableRegistry humanoidHighLevelControllerManagerRegistry = new YoVariableRegistry("HumanoidHighLevelControllerManager");
      YoVariableRegistry managerParentRegistry = new YoVariableRegistry("HighLevelHumanoidControllerFactory");
      YoVariableRegistry walkingParentRegistry = new YoVariableRegistry("WalkingControllerState");
      registry.addChild(drcControllerThreadRegistry);
      drcControllerThreadRegistry.addChild(drcMomentumBasedControllerRegistry);
      drcMomentumBasedControllerRegistry.addChild(humanoidHighLevelControllerManagerRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(walkingParentRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(managerParentRegistry);

      controllerToolbox = createHighLevelControllerToolbox(robotModel, yoGraphicsListRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(controllerToolbox.getYoVariableRegistry());
      setupWalkingMessageHandler(walkingControllerParameters, capturePointPlannerParameters, yoGraphicsListRegistry);
      rootJoint = fullRobotModel.getRootJoint();

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      KinematicsToolboxHelper.setRobotStateFromRawData(initialPelvisPose,
                                                       initialJointAngles,
                                                       rootJoint,
                                                       FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel));

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      managerFactory = new HighLevelControlManagerFactory(managerParentRegistry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      walkingController = new WalkingHighLevelHumanoidController(walkingInputManager,
                                                                 walkingOutputManager,
                                                                 managerFactory,
                                                                 walkingControllerParameters,
                                                                 controllerToolbox);
      walkingParentRegistry.addChild(walkingController.getYoVariableRegistry());

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      WholeBodyControlCoreToolbox controlCoreToolbox = createControllerCoretoolbox(walkingControllerParameters, yoGraphicsListRegistry);

      FeedbackControlCommandList feedbackControlTemplate = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(controllerToolbox.getControlledOneDoFJoints());

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackControlTemplate, jointDesiredOutputList, walkingParentRegistry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      double controlDT = controllerToolbox.getControlDT();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      YoDouble yoTime = controllerToolbox.getYoTime();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
      linearMomentumRateControlModule = new LinearMomentumRateControlModule(referenceFrames,
                                                                            contactableFeet,
                                                                            elevator,
                                                                            walkingControllerParameters,
                                                                            yoTime,
                                                                            gravityZ,
                                                                            controlDT,
                                                                            walkingParentRegistry,
                                                                            yoGraphicsListRegistry);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      ParameterLoaderHelper.loadParameters(this, robotModel, drcControllerThreadRegistry);

      YoVariable<?> defaultHeight = registry.getVariable(PelvisHeightControlState.class.getSimpleName(),
                                                         PelvisHeightControlState.class.getSimpleName() + "DefaultHeight");
      if (Double.isNaN(defaultHeight.getValueAsDouble()))
      {
         throw new RuntimeException("Need to load a default height.");
      }
   }

   private HighLevelHumanoidControllerToolbox createHighLevelControllerToolbox(DRCRobotModel robotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double omega0 = robotModel.getWalkingControllerParameters().getOmega0();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = createContactableBodiesFactory(robotModel);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();

      List<ContactablePlaneBody> allContactableBodies = new ArrayList<>(additionalContacts);
      allContactableBodies.addAll(feet.values());

      double robotMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      for (RobotSide robotSide : RobotSide.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(feet.get(robotSide), robotMass, 2, registry);
         footSwitch.setFootContactState(true);
         footSwitches.put(robotSide, footSwitch);
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      return new HighLevelHumanoidControllerToolbox(fullRobotModel,
                                                    referenceFrames,
                                                    footSwitches,
                                                    null,
                                                    yoTime,
                                                    gravityZ,
                                                    omega0,
                                                    feet,
                                                    integrationDT,
                                                    Collections.emptyList(),
                                                    allContactableBodies,
                                                    yoGraphicsListRegistry,
                                                    jointsToIgnore);
   }

   private void setupWalkingMessageHandler(WalkingControllerParameters walkingControllerParameters, ICPPlannerParameters icpPlannerParameters,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      double defaultSwingDurationShiftFraction = icpPlannerParameters.getSwingDurationShiftFraction();
      double defaultSwingSplitFraction = icpPlannerParameters.getSwingSplitFraction();
      double defaultTransferSplitFraction = icpPlannerParameters.getTransferSplitFraction();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime,
                                                                              defaultSwingTime,
                                                                              defaultInitialTransferTime,
                                                                              defaultFinalTransferTime,
                                                                              defaultSwingDurationShiftFraction,
                                                                              defaultSwingSplitFraction,
                                                                              defaultTransferSplitFraction,
                                                                              defaultTransferSplitFraction,
                                                                              controllerToolbox.getContactableFeet(),
                                                                              walkingOutputManager,
                                                                              yoTime,
                                                                              yoGraphicsListRegistry,
                                                                              controllerToolbox.getYoVariableRegistry());
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);
   }

   private WholeBodyControlCoreToolbox createControllerCoretoolbox(WalkingControllerParameters walkingControllerParameters,
                                                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      JointBasics[] controlledJoints = controllerToolbox.getControlledJoints();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      FeedbackControllerSettings feedbackControllerSettings = walkingControllerParameters.getFeedbackControllerSettings();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(integrationDT,
                                                                                       gravityZ,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       controlledJoints,
                                                                                       controllerToolbox.getCenterOfMassFrame(),
                                                                                       momentumOptimizationSettings,
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);

      controlCoreToolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      controlCoreToolbox.setFeedbackControllerSettings(feedbackControllerSettings);
      controlCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());

      return controlCoreToolbox;
   }

   private ContactableBodiesFactory<RobotSide> createContactableBodiesFactory(DRCRobotModel robotModel)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionaContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionaContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);

      return contactableBodiesFactory;
   }

   public void initialize()
   {
      zeroMotion();

      referenceFrames.updateFrames();
      controllerCore.initialize();
      controllerToolbox.update();
      walkingController.initialize();

      taskExecutor.clear();
      taskExecutor.submit(new KinematicsWalkingInitializeTask(walkingController, controllerToolbox.getFootContactStates())); // few ticks to get it going
   }

   public void doControl()
   {
      if (kinematicsSimulationInputManager.isNewCommandAvailable(FootstepDataListCommand.class))
      {
         FootstepDataListCommand foostepCommand = kinematicsSimulationInputManager.pollNewestCommand(FootstepDataListCommand.class);
         taskExecutor.submit(new KinematicsWalkingFootstepSequenceTask(fullRobotModel.getRootJoint(),
                                                                       foostepCommand,
                                                                       walkingInputManager,
                                                                       walkingOutputManager,
                                                                       controllerToolbox.getFootContactStates(),
                                                                       managerFactory.getOrCreateBalanceManager(),
                                                                       footSwitches,
                                                                       walkingStatusPublisher,
                                                                       footstepStatusPublisher));
      }
      else
      {
         yoTime.add(integrationDT);
         fullRobotModel.updateFrames();
         referenceFrames.updateFrames();
         controllerToolbox.update();
      }

      if (taskExecutor.isDone())  // keep robot from drifting when no tasks are present
      {
         zeroMotion();
         return;
      }

      taskExecutor.doControl();

      walkingController.doAction();

      linearMomentumRateControlModule.setInputFromWalkingStateMachine(walkingController.getLinearMomentumRateControlModuleInput());
      if (!linearMomentumRateControlModule.computeControllerCoreCommands())
      {
         controllerToolbox.reportControllerFailureToListeners(new FrameVector2D());
      }
      walkingController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());
      if (!taskExecutor.isDone())
         controllerCoreCommand.addInverseDynamicsCommand(((KinematicsWalkingTask) taskExecutor.getCurrentTask()).getOutput());

      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      linearMomentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
      linearMomentumRateControlModule.computeAchievedCMP();

      rootJoint.setJointAcceleration(0, controllerCore.getOutputForRootJoint().getDesiredAcceleration());
      JointDesiredOutputListReadOnly jointDesiredOutputList = controllerCore.getOutputForLowLevelController();

      for (OneDoFJointBasics joint : controllerToolbox.getControlledOneDoFJoints())
      {
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         joint.setQdd(jointDesiredOutput.getDesiredAcceleration());
      }

      integrator.setIntegrationDT(integrationDT);
      integrator.doubleIntegrateFromAcceleration(Arrays.asList(controllerToolbox.getControlledJoints()));
   }

   private void zeroMotion()
   {
      for (JointBasics joint : fullRobotModel.getElevator().childrenSubtreeIterable())
      {
         joint.setJointAccelerationToZero();
         joint.setJointTwistToZero();
      }
   }

   public CommandInputManager getInputManager()
   {
      return kinematicsSimulationInputManager;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
