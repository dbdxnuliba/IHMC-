package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CollisionManager
{
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final RigidBodyBasics body;
   MovingReferenceFrame firstEndLinkFrame, otherEndLinkFrame;
   FramePose3D firstEndPose = new FramePose3D();
   FramePose3D otherEndPose = new FramePose3D();

   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble bodyOriginX, bodyOriginY, bodyOriginZ;
   private final YoDouble bodyEndX, bodyEndY, bodyEndZ;
   private final YoDouble measuredDistance, expectedLength;

   public CollisionManager(MovingReferenceFrame firstEndLinkFrame, MovingReferenceFrame otherEndLinkFrame, RigidBodyBasics body,
                           RigidBodyBasics elevator, YoVariableRegistry parentRegistry, double expectedLength)
   {
      spatialAccelerationCommand.set(/*
                                      * MultiBodySystemTools.getRootBody(body)
                                      */elevator, body); //Does this uses also the base acceleration?
      spatialAccelerationCommand.getWeightMatrix().setLinearWeights(10.0, 10.0, 10.0);
      this.firstEndLinkFrame = firstEndLinkFrame;
      this.otherEndLinkFrame = otherEndLinkFrame;
      this.body = body;
      parentRegistry.addChild(registry);
      bodyOriginX = new YoDouble(body.getName() + "_originX", registry);
      bodyOriginY = new YoDouble(body.getName() + "_originY", registry);
      bodyOriginZ = new YoDouble(body.getName() + "_originZ", registry);
      bodyEndX = new YoDouble(body.getName() + "_endX", registry);
      bodyEndY = new YoDouble(body.getName() + "_endY", registry);
      bodyEndZ = new YoDouble(body.getName() + "_endZ", registry);

      measuredDistance = new YoDouble(body.getName() + "measuredLenght", registry);
      this.expectedLength = new YoDouble(body.getName() + "expectedLenght", registry);
      this.expectedLength.set(expectedLength);
   }

   public void compute(boolean loadBearing)
   {
      
      firstEndPose.setToZero(firstEndLinkFrame);
      firstEndPose.changeFrame(ReferenceFrame.getWorldFrame());

      bodyOriginX.set(firstEndPose.getX());
      bodyOriginY.set(firstEndPose.getY());
      bodyOriginZ.set(firstEndPose.getZ());

      otherEndPose.setToZero(otherEndLinkFrame);
      otherEndPose.changeFrame(ReferenceFrame.getWorldFrame());

      bodyEndX.set(otherEndPose.getX());
      bodyEndY.set(otherEndPose.getY());
      bodyEndZ.set(otherEndPose.getZ());

      measuredDistance.set(otherEndPose.getPositionDistance(firstEndPose.getPosition()));

      
//      if (loadBearing)
      {
         spatialAccelerationCommand.getSelectionMatrix().clearSelection();
      }
//      else
//      {
//         desiredLinearAcceleration.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 10.0, 0.0);
//         spatialAccelerationCommand.getSelectionMatrix().clearAngularSelection();
      //      spatialAccelerationCommand.getSelectionMatrix().getLinearPart().resetSelection();
//         desiredLinearAcceleration.changeFrame(body.getBodyFixedFrame());
//         spatialAccelerationCommand.setLinearAcceleration(body.getBodyFixedFrame(), desiredLinearAcceleration);
//      }
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

}
