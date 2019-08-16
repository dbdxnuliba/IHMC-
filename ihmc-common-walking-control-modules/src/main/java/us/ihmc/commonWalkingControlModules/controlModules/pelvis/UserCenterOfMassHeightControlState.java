package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final AbstractPDController linearMomentumZPDController;
   private final SymmetricPID3DGains symmetric3DGains = new SymmetricPID3DGains();
   private final CenterOfMassTrajectoryHandler comTrajectoryHandler;
   private final MovingReferenceFrame centerOfMassFrame;
   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint3D desiredCenterOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FrameVector3D desiredCenterOfMassVelocity = new FrameVector3D();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble yoTime;


   public UserCenterOfMassHeightControlState(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      comTrajectoryHandler = controllerToolbox.getWalkingMessageHandler().getComTrajectoryHandler();
      centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      yoTime = controllerToolbox.getYoTime();

      DoubleProvider proportionalGain = () -> symmetric3DGains.getProportionalGains()[2];
      DoubleProvider derivativeGain = () -> symmetric3DGains.getDerivativeGains()[2];

      linearMomentumZPDController = AbstractPDController.createPDController("userCoMHeightControlState_linearMomentumZPDController",
                                                                            proportionalGain,
                                                                            derivativeGain,
                                                                            () -> 0.0,
                                                                            registry);

      parentRegistry.addChild(registry);
   }

   public void setGains(PIDGainsReadOnly gains)
   {
      symmetric3DGains.setGains(gains);
   }

   @Override
   public void doAction(double timeInState)
   {
      return;
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
   }

   @Override
   public void goHome(double trajectoryTime)
   {
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
   }

   @Override
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
                                                     FeetManager feetManager)
   {
      centerOfMassVelocity.setIncludingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());
      centerOfMassPosition.setToZero(centerOfMassFrame);

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      comTrajectoryHandler.packDesiredCoMState(yoTime.getDoubleValue(), desiredCenterOfMassPosition, desiredCenterOfMassVelocity);

      return linearMomentumZPDController.compute(centerOfMassPosition.getZ(),
                                                 desiredCenterOfMassPosition.getZ(),
                                                 centerOfMassVelocity.getZ(),
                                                 desiredCenterOfMassVelocity.getZ());
   }

   public boolean isCoMHeightTrajectoryAvailable()
   {
      return comTrajectoryHandler.isWithinInterval(yoTime.getDoubleValue());
   }

}
