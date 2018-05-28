package us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CentroidalRobotEstimator implements RobotController
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry("EstimatorRegistry");
   private final FloatingJoint rootJoint;
   private CentroidalState state;
   private final YoFramePose pose;
   private final PoseReferenceFrame poseFrame;
   private final String namePrefix;

   // Variables for cals 
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FrameQuaternion tempQuaternion = new FrameQuaternion();

   public CentroidalRobotEstimator(String robotName, FloatingJoint rootJoint, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.namePrefix = robotName + "Estimated";
      this.rootJoint = rootJoint;
      pose = new YoFramePose(namePrefix + "Pose", worldFrame, registry);
      poseFrame = new PoseReferenceFrame(namePrefix + "Pose", worldFrame);
      if (graphicsListRegistry != null)
      {
         YoGraphicCoordinateSystem poseGraphic = new YoGraphicCoordinateSystem(namePrefix + "PoseGraphic", pose, 0.1);
         YoFramePoint2d yoPoint = new YoFramePoint2d(pose.getYoX(), pose.getYoY(), pose.getReferenceFrame());
         YoArtifactPosition positionArtifact = new YoArtifactPosition(namePrefix + "CoM", yoPoint, GraphicType.BALL_WITH_CROSS, Color.BLACK, 0.002);
         graphicsListRegistry.registerYoGraphic("EstimatorGraphicsList", poseGraphic);
         graphicsListRegistry.registerArtifact("EstimatorArtifactList", positionArtifact);
      }
   }

   public void setStateToUpdate(CentroidalState stateToUpdate)
   {
      this.state = stateToUpdate;
   }

   @Override
   public void initialize()
   {
      if(state == null)
         throw new RuntimeException("State object to update has not been set");
      doControl();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "CentroidalRobotEstimator";
   }

   @Override
   public String getDescription()
   {
      return "Provides an estimation of the robot's current pose";
   }

   @Override
   public void doControl()
   {
      rootJoint.getPosition(tempPoint);
      rootJoint.getQuaternion(tempQuaternion);
      poseFrame.setPoseAndUpdate(tempPoint, tempQuaternion);
      pose.set(tempPoint, tempQuaternion);

      state.setPosition(tempPoint);
      state.setOrientation(tempQuaternion);

      rootJoint.getVelocity(tempVector);
      state.setLinearVelocity(tempVector);

      rootJoint.getLinearAcceleration(tempVector);
      state.setLinearAcceleration(tempVector);

      rootJoint.getAngularVelocity(tempVector, poseFrame);
      tempVector.changeFrame(worldFrame);
      state.setAngularVelocity(tempVector);

      rootJoint.getAngularAcceleration(tempVector, poseFrame);
      tempVector.changeFrame(worldFrame);
      state.setAngularAcceleration(tempVector);
   }

   public CentroidalState getState()
   {
      return state;
   }

   public void getState(CentroidalState stateToSet)
   {
      stateToSet.set(state);
   }
}
