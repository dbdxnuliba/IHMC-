package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.List;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxCalibrationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
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

   private Pose3D operatorHeadPose;
   private final SideDependentList<Pose3D> operatorHandPoses = new SideDependentList<>();
   private double operatorGroundHeight;

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

   private double operatorHeight;
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

      operatorHeadPose = null;
      operatorHandPoses.clear();
      operatorGroundHeight = Double.NaN;
   }

   @Override
   public void doAction(double timeInState)
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxCalibrationCommand.class))
      {
         List<KinematicsStreamingToolboxCalibrationCommand> commands = commandInputManager.pollNewCommands(KinematicsStreamingToolboxCalibrationCommand.class);

         for (KinematicsStreamingToolboxCalibrationCommand command : commands)
         {
            if (command.hasGroundHeight())
               operatorGroundHeight = command.getGroundHeight();
            if (command.hasHeadPose())
               operatorHeadPose = new Pose3D(command.getHeadPose());
            for (RobotSide robotSide : RobotSide.values)
            {
               if (command.hasHandPose(robotSide))
                  operatorHandPoses.put(robotSide, new Pose3D(command.getHandPose(robotSide)));
            }
         }
      }

      if (hasAllInputs())
      {
         KSTTools.updateFullRobotModel(tools.getRobotConfigurationData(), currentFullRobotModel);
         currentReferenceFrames.updateFrames();

         Point3D operatorMidFootZUpPosition = operatorMidFootZUpPose.getPosition();
         operatorMidFootZUpPosition.setX(operatorHeadPose.getX());
         operatorMidFootZUpPosition.setY(operatorHeadPose.getY());
         operatorMidFootZUpPosition.setZ(operatorGroundHeight);
         operatorMidFootZUpPose.getOrientation().setToYawOrientation(operatorHeadPose.getYaw());
         operatorMidFootZUpGroundFrame.update();

         operatorHeight = operatorHeadPose.getZ() - operatorGroundHeight;
         userInputTransform.setOperatorToRobotHeightScale(robotHeight / operatorHeight);

         operatorHeadHandDistance = 0.0;

         for (RobotSide robotSide : RobotSide.values)
            operatorHeadHandDistance += 0.5 * operatorHeadPose.getPositionDistance(operatorHandPoses.get(robotSide));
         userInputTransform.setOperatorToRobotArmScale(robotHeadHandDistance / operatorHeadHandDistance);

         userInputTransform.setOperatorMidFootZupGroundFrame(operatorMidFootZUpGroundFrame);
         userInputTransform.setRobotMidFootZUpGroundFrame(currentMidFootZUpGroundFrame);

         isCalibrated.set(true);
      }
   }

   private boolean hasAllInputs()
   {
      if (operatorHeadPose == null)
         return false;
      if (operatorHandPoses.get(RobotSide.LEFT) == null || operatorHandPoses.get(RobotSide.RIGHT) == null)
         return false;
      if (Double.isNaN(operatorGroundHeight))
         return false;
      return true;
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
