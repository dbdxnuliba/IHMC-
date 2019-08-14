package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxCalibrationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class KSTCalibrationState implements State
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final double armSpanToHeightRatio = 1.135;

   private final YoBoolean isCalibrated;
   private final KSTTools tools;
   private final CommandInputManager commandInputManager;
   private final FullHumanoidRobotModel currentFullRobotModel;

   private final HumanoidReferenceFrames currentReferenceFrames;

   private final Pose3D operatorMidFootZUpPose = new Pose3D();

   private MovingReferenceFrame currentMidFootZUpGroundFrame;
   private final ReferenceFrame operatorMidFootZUpGroundFrame = new ReferenceFrame("operatorMidFootZUpGroundFrame", worldFrame, true, true)
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(operatorMidFootZUpPose.getOrientation(), operatorMidFootZUpPose.getPosition());
      }
   };

   private final double robotHeight;

   private double operatorHeadHandDistance;
   private final double robotHeadHandDistance;

   private final KSTUserInputTransform userInputTransform;

   public KSTCalibrationState(KSTTools tools)
   {
      this.tools = tools;
      userInputTransform = tools.getUserInputTransform();
      commandInputManager = tools.getCommandInputManager();
      YoVariableRegistry registry = tools.getRegistry();

      isCalibrated = new YoBoolean("isCalibrated", registry);

      currentFullRobotModel = tools.getFullRobotModelFactory().createFullRobotModel();
      FullHumanoidRobotModel zeroPoseFullRobotModel = tools.getFullRobotModelFactory().createFullRobotModel();

      currentReferenceFrames = new HumanoidReferenceFrames(currentFullRobotModel);
      HumanoidReferenceFrames zeroPoseReferenceFrames = new HumanoidReferenceFrames(zeroPoseFullRobotModel);
      zeroPoseReferenceFrames.updateFrames();

      MovingReferenceFrame zeroPoseMidFootZUpGroundFrame = zeroPoseReferenceFrames.getMidFootZUpGroundFrame();
      currentMidFootZUpGroundFrame = currentReferenceFrames.getMidFootZUpGroundFrame();

      FramePoint3D zeroPoseHeadPosition = new FramePoint3D(zeroPoseFullRobotModel.getHead().getBodyFixedFrame());
      zeroPoseHeadPosition.changeFrame(zeroPoseMidFootZUpGroundFrame);
      robotHeight = zeroPoseHeadPosition.getZ();

      FramePoint3D zeroPoseHandPosition = new FramePoint3D(zeroPoseFullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame());
      zeroPoseHandPosition.changeFrame(zeroPoseMidFootZUpGroundFrame);
      Vector3D headToHand = new Vector3D();
      headToHand.sub(zeroPoseHandPosition, zeroPoseHeadPosition);
      robotHeadHandDistance = headToHand.length();
   }

   @Override
   public void onEntry()
   {
      tools.getIKController().getDefaultGains().setMaxFeedbackAndFeedbackRate(1200.0, Double.POSITIVE_INFINITY);
   }

   @Override
   public void doAction(double timeInState)
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxCalibrationCommand.class))
      {
         KinematicsStreamingToolboxCalibrationCommand command = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxCalibrationCommand.class);

         Pose3D operatorHeadPose = command.getHeadPose();
         SideDependentList<Pose3D> operatorHandPoses = command.getHandPoses();

         KSTTools.updateFullRobotModel(tools.getRobotConfigurationData(), currentFullRobotModel);
         currentReferenceFrames.updateFrames();
         double operatorHeight = operatorHandPoses.get(RobotSide.LEFT).getPositionDistance(operatorHandPoses.get(RobotSide.RIGHT)) / armSpanToHeightRatio;

         Point3D operatorMidFootZUpPosition = operatorMidFootZUpPose.getPosition();
         operatorMidFootZUpPosition.set(operatorHeadPose.getPosition());
         operatorMidFootZUpPosition.subZ(operatorHeight);
         operatorMidFootZUpPose.getOrientation().setToYawOrientation(operatorHeadPose.getYaw());
         operatorMidFootZUpGroundFrame.update();

         double operatorToRobotHeightScale = robotHeight / operatorHeight;
         userInputTransform.setOperatorToRobotHeightScale(operatorToRobotHeightScale);

         operatorHeadHandDistance = 0.0;

         for (RobotSide robotSide : RobotSide.values)
            operatorHeadHandDistance += 0.5 * operatorHeadPose.getPositionDistance(operatorHandPoses.get(robotSide));
         userInputTransform.setOperatorToRobotArmScale(robotHeadHandDistance / operatorHeadHandDistance);

         userInputTransform.setOperatorMidFootZupGroundFrame(operatorMidFootZUpGroundFrame);
         userInputTransform.setRobotMidFootZUpGroundFrame(currentMidFootZUpGroundFrame);

         FramePoint3D framePoint3D = new FramePoint3D(operatorMidFootZUpGroundFrame);
         framePoint3D.changeFrame(currentMidFootZUpGroundFrame);
         FrameVector3D frameVector3D = new FrameVector3D(framePoint3D);
         frameVector3D.changeFrame(worldFrame);
         LogTools.info("Calibration result: height scale = " + operatorToRobotHeightScale + ", arm length scale = "
               + (robotHeadHandDistance / operatorHeadHandDistance) + ", operator world offset = " + frameVector3D);

         isCalibrated.set(true);
      }
   }

   @Override
   public void onExit()
   {
      isCalibrated.set(false);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isCalibrated.getValue();
   }
}
