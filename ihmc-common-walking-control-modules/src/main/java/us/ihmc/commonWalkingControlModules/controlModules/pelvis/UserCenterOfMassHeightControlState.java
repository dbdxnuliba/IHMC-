package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.controllers.AbstractPDController;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricPID3DGains;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class UserCenterOfMassHeightControlState implements PelvisAndCenterOfMassHeightControlState
{
   /** Handles the trajectory and the queuing. It is used as a hack  **/
   private final UserCenterOfMassHeightController positionController;

   private final TaskspaceTrajectoryStatusMessageHelper statusHelper = new TaskspaceTrajectoryStatusMessageHelper("centerOfMassHeight");

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final AbstractPDController linearMomentumZPDController;
   private final SymmetricPID3DGains symmetric3DGains = new SymmetricPID3DGains();
   private final MovingReferenceFrame centerOfMassFrame;
   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint3D desiredCenterOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FrameVector3D desiredCenterOfMassVelocity = new FrameVector3D();
   private final FrameVector3D desiredCenterOfMassAcceleration = new FrameVector3D();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble yoTime;
   private final YoDouble desiredCenterOfMassHeightFromController, desiredCenterOfMassHeightFromManager;

   private final CenterOfMassTrajectoryHandler comHandler;

   public UserCenterOfMassHeightControlState(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      yoTime = controllerToolbox.getYoTime();

      //This controller is used only to deal with the handling and queuing of trajectories.
      positionController = new UserCenterOfMassHeightController(centerOfMassFrame, yoTime, registry);

      DoubleProvider proportionalGain = () -> symmetric3DGains.getProportionalGains()[2];
      DoubleProvider derivativeGain = () -> symmetric3DGains.getDerivativeGains()[2];
      linearMomentumZPDController = AbstractPDController.createPDController("userCoMHeightControlState_linearMomentumZPDController",
                                                                            proportionalGain,
                                                                            derivativeGain,
                                                                            () -> 0.0,
                                                                            registry);

      parentRegistry.addChild(registry);
      
      comHandler = controllerToolbox.getWalkingMessageHandler().getComTrajectoryHandler();
      desiredCenterOfMassHeightFromController = new YoDouble("DesiredCoMHeight_controller", registry);
      desiredCenterOfMassHeightFromManager = new YoDouble("DesiredCoMHeight_manager", registry);

   }

   public void setGains(PIDGainsReadOnly gains)
   {
      symmetric3DGains.setGains(gains);
   }

   @Override
   public void doAction(double timeInState)
   {
      positionController.doAction(Double.NaN);
      statusHelper.updateWithTimeInTrajectory(positionController.getTimeInTrajectory());
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      positionController.holdCurrent();
   }

   @Override
   public void goHome(double trajectoryTime)
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);
      positionController.goToPositionFromCurrent(centerOfMassPosition, trajectoryTime);
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (command.isStopAllTrajectory())
      {
         initializeDesiredHeightToCurrent();
      }
   }

   public boolean handleCenterOfMassTrajectoryCommand(CenterOfMassTrajectoryCommand command)
   {
      EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();

      if (positionController.handleTrajectoryCommand(euclideanTrajectory))
      {
         euclideanTrajectory.setSequenceId(command.getSequenceId());
         statusHelper.registerNewTrajectory(euclideanTrajectory);
         return true;
      }

      initializeDesiredHeightToCurrent();
      return false;
   }

   @Override
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
                                                     FeetManager feetManager)
   {
      centerOfMassVelocity.setIncludingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());
      centerOfMassPosition.setToZero(centerOfMassFrame);

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      boolean ok = comHandler.packDesiredCoMState(yoTime.getDoubleValue(), desiredCenterOfMassPosition, desiredCenterOfMassVelocity, desiredCenterOfMassAcceleration);

      desiredCenterOfMassPosition.set(positionController.getDesiredPosition());
      desiredCenterOfMassVelocity.set(positionController.getDesiredVelocity());
      desiredCenterOfMassAcceleration.set(positionController.getFeedForwardAcceleration());

      if (!ok)
      {
         return 0.0;
      }
      
      desiredCenterOfMassHeightFromController.set(positionController.getDesiredPosition().getZ());
      desiredCenterOfMassHeightFromManager.set(desiredCenterOfMassPosition.getZ());

      return desiredCenterOfMassAcceleration.getZ() + linearMomentumZPDController.compute(centerOfMassPosition.getZ(),
                                                                                          desiredCenterOfMassPosition.getZ(),
                                                                                          centerOfMassVelocity.getZ(),
                                                                                          desiredCenterOfMassVelocity.getZ());
   }

   public boolean isCoMHeightTrajectoryAvailable()
   {
      return comHandler.isWithinInterval(yoTime.getDoubleValue());
   }

}
