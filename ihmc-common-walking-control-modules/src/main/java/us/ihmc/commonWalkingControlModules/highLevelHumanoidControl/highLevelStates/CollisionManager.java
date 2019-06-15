package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CollisionManagerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollisionManager
{
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PointFeedbackControlCommand pointFeedbackCommand = new PointFeedbackControlCommand();

   private final RigidBodyBasics body;
   RigidBodyBasics root;
   private final ReferenceFrame firstEndLinkFrame, otherEndLinkFrame;
   private final FramePose3D firstEndPose = new FramePose3D();
   private final FramePose3D otherEndPose = new FramePose3D();
   private final FramePose3D firstEndPoseInPlaneCoordinates = new FramePose3D();
   private final FramePose3D otherEndPoseInPlaneCoordinates = new FramePose3D();
   private final FramePose3D body_H_closestPoint = new FramePose3D();
   private final Line3D bodyLine = new Line3D();
   private final RigidBodyTransform planeToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform planeFromWorldTransform = new RigidBodyTransform();
   private final Point3D firstMinDistanceSegmentPointInPlaneCoordinates = new Point3D();
   private final Point3D secondMinDistanceSegmentPointInPlaneCoordinates = new Point3D();
   private final Point3D firstMinDistanceSegmentPoint = new Point3D();
   private final Point3D secondMinDistanceSegmentPoint = new Point3D();
   private final Point3D firstConcaveHullVertex = new Point3D();
   private final Point3D secondConcaveHullVertex = new Point3D();
   private final FrameVector3D minDistanceVector = new FrameVector3D();
   private final FrameVector3D zAxis = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
   private final FramePoint3D closestPointOnBody = new FramePoint3D();
   private final AxisAngle worldZToDistanceVectorRotation = new AxisAngle();
   private final LineSegment2D edge = new LineSegment2D();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D zeroVector = new FrameVector3D();

   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(100, PlanarRegion.class);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble bodyOriginX, bodyOriginY, bodyOriginZ;
   private final YoDouble bodyEndX, bodyEndY, bodyEndZ;
   private final YoDouble distanceX, distanceY, distanceZ;
   private final YoDouble closestBodyPointX, closestBodyPointY, closestBodyPointZ;
   private final YoInteger closestPlanarRegion;
   private final YoDouble measuredDistance, expectedLength, minimumDistanceValue, desiredAcceleration;
   private final YoInteger numberOfPlanarSurfaces;

   public CollisionManager(ReferenceFrame firstEndLinkFrame, ReferenceFrame otherEndLinkFrame, RigidBodyBasics body,
                           RigidBodyBasics elevator, YoVariableRegistry parentRegistry)
   {
      spatialAccelerationCommand.set(/*
                                      * MultiBodySystemTools.getRootBody(body)
                                      */
                                     elevator,
                                     body); //Does this uses also the base acceleration?
      
      pointFeedbackCommand.set(elevator, body);
      pointFeedbackCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      this.firstEndLinkFrame = firstEndLinkFrame;
      this.otherEndLinkFrame = otherEndLinkFrame;
      this.body = body;
      root = elevator;
      parentRegistry.addChild(registry);
      bodyOriginX = new YoDouble("collision_" + body.getName() + "_originX", registry);
      bodyOriginY = new YoDouble("collision_" + body.getName() + "_originY", registry);
      bodyOriginZ = new YoDouble("collision_" + body.getName() + "_originZ", registry);
      bodyEndX = new YoDouble("collision_" + body.getName() + "_endX", registry);
      bodyEndY = new YoDouble("collision_" + body.getName() + "_endY", registry);
      bodyEndZ = new YoDouble("collision_" + body.getName() + "_endZ", registry);
      distanceX = new YoDouble("collision_" + body.getName() + "_minDistanceDirectionX", registry);
      distanceY = new YoDouble("collision_" + body.getName() + "_minDistanceDirectionY", registry);
      distanceZ = new YoDouble("collision_" + body.getName() + "_minDistanceDirectionZ", registry);
      closestBodyPointX = new YoDouble("collision_" + body.getName() + "_closestBodyPointX", registry);
      closestBodyPointY = new YoDouble("collision_" + body.getName() + "_closestBodyPointY", registry);
      closestBodyPointZ = new YoDouble("collision_" + body.getName() + "_closestBodyPointZ", registry);
      minimumDistanceValue = new YoDouble("collision_" + body.getName() + "_minDistanceValue", registry);
      closestPlanarRegion = new YoInteger("collision_" + body.getName() + "_closestRegion", registry);
      desiredAcceleration = new YoDouble("collision_" + body.getName() + "_desiredAccelerationModule", registry);

      measuredDistance = new YoDouble("collision_" + body.getName() + "measuredLenght", registry);
      this.expectedLength = new YoDouble("collision_" + body.getName() + "expectedLenght", registry);
      
      numberOfPlanarSurfaces = new YoInteger("collision_numberOfPlanarSurfaces", registry);
      
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

      bodyLine.set(firstEndPose.getPosition(), otherEndPose.getPosition());

      double distanceThreshold = 0.1;

      double minDistance = distanceThreshold;
      FrameVector3D distanceVector = new FrameVector3D();
      FramePoint3D pointOnBody = new FramePoint3D();

      for (int i = 0; i < planarRegions.size(); ++i)
      {
         double distance = computeDistanceFromPlanarRegion(planarRegions.get(i), distanceVector, pointOnBody);

         if (distance < minDistance)
         {
            minDistance = distance;
            minDistanceVector.set(distanceVector);
            closestPointOnBody.set(pointOnBody);
            closestPlanarRegion.set(i);
         }
      }
      
      double maxZComponent = -0.1;
      
      if (minDistanceVector.getZ() > maxZComponent)
      {
         minDistanceVector.setZ(maxZComponent);
         minDistanceVector.normalize();
      }
      
      distanceX.set(minDistanceVector.getX());
      distanceY.set(minDistanceVector.getY());
      distanceZ.set(minDistanceVector.getZ());
      closestBodyPointX.set(closestPointOnBody.getX());
      closestBodyPointY.set(closestPointOnBody.getY());
      closestBodyPointZ.set(closestPointOnBody.getZ());
      minimumDistanceValue.set(minDistance);

      setupCommands(distanceThreshold, minDistance, distanceVector);
   }

   private void setupCommands(double distanceThreshold, double minDistance, FrameVector3D distanceVector)
   {
      if (minDistance >= 0.0 && minDistance < distanceThreshold)
      {
         EuclidFrameTools.axisAngleFromFirstToSecondVector3D(zAxis, minDistanceVector, worldZToDistanceVectorRotation);

         body_H_closestPoint.setToZero(ReferenceFrame.getWorldFrame());
         body_H_closestPoint.setPosition(closestPointOnBody);
         body_H_closestPoint.setOrientation(worldZToDistanceVectorRotation); //This frame is located on the point of
                                                                          //the body at the minimum distance from the closest obstacle 
                                                                          //and oriented such that the Z-axis is aligned 
                                                                          //with the distance direction
         body_H_closestPoint.changeFrame(body.getBodyFixedFrame()); //Express the pose in body coordinates

         RigidBodyTransform body_H_closestPointAsRBT = new RigidBodyTransform(body_H_closestPoint.getOrientation(), body_H_closestPoint.getPosition());

         ReferenceFrame closestPointFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(body.getName() + "_collisionFrame",
                                                                                                                body.getBodyFixedFrame(),
                                                                                                                body_H_closestPointAsRBT);

         desiredLinearAcceleration.setToZero(closestPointFrame);

         double accelerationGain = 500.0;
         double maxAcceleration = 50.0;

         double desiredAccelerationValue = accelerationGain * (distanceThreshold
               - minDistance);/*
                               * (1.0 / (minDistance + 1e-4) - 1.0 /
                               * distanceThreshold) / (minDistance * minDistance
                               * + 1e-4);
                               */

         desiredLinearAcceleration.setZ(saturate(desiredAccelerationValue, maxAcceleration));

         spatialAccelerationCommand.getSelectionMatrix().clearSelection();
         //         spatialAccelerationCommand.getSelectionMatrix().getLinearPart().selectZAxis(true);
         spatialAccelerationCommand.getSelectionMatrix().getLinearPart().setSelectionFrame(closestPointFrame);
         spatialAccelerationCommand.getSelectionMatrix().getLinearPart().setAxisSelection(false, false, true);

         spatialAccelerationCommand.getWeightMatrix().setWeightFrame(closestPointFrame);
         spatialAccelerationCommand.getWeightMatrix().setLinearWeights(0.0, 0.0, 10.0);
         //TODO Figure out how to use the control frame
         //         spatialAccelerationCommand.getControlFramePose().set(other);

//         pointFeedbackCommand.getSpatialAccelerationCommand().set(spatialAccelerationCommand);
         SelectionMatrix3D selection = new SelectionMatrix3D(closestPointFrame, false, false, true);
         pointFeedbackCommand.setSelectionMatrix(selection);

         WeightMatrix3D weights = new WeightMatrix3D();
         weights.setWeightFrame(closestPointFrame);
         weights.setWeights(0.0, 0.0, 10.0);
         pointFeedbackCommand.setWeightMatrix(weights);
         pointFeedbackCommand.setBodyFixedPointToControl(body_H_closestPoint.getPosition());
         //         pointFeedbackCommand.setGainsFrame(closestPointFrame);
         pointFeedbackCommand.getGains().setProportionalGains(accelerationGain);
         pointFeedbackCommand.getGains().setDerivativeGains(2.0 * Math.sqrt(accelerationGain));
         pointFeedbackCommand.getGains().setIntegralGains(0.0, 0.0);
         pointFeedbackCommand.getGains().setMaxFeedbackAndFeedbackRate(maxAcceleration, maxAcceleration * 10);

         //         feedForwardLinearAcceleration.setIncludingFrame(pointFeedbackCommand.getReferenceLinearAcceleration());
         //         feedForwardLinearAcceleration.setX(0.0);
         //         feedForwardLinearAcceleration.setY(0.0);
         //         feedForwardLinearAcceleration.setZ(0.0);
         //         pointFeedbackCommand.getReferenceLinearAcceleration().set(feedForwardLinearAcceleration);

         desiredPosition.set(closestPointOnBody.getX() - (distanceThreshold - minDistance) * distanceVector.getX(),
                             closestPointOnBody.getY() - (distanceThreshold - minDistance) * distanceVector.getY(),
                             closestPointOnBody.getZ() - (distanceThreshold - minDistance) * distanceVector.getZ());
         desiredPosition.changeFrame(root.getBodyFixedFrame());

         pointFeedbackCommand.setInverseDynamics(desiredPosition, zeroVector, zeroVector);

         spatialAccelerationCommand.setLinearAcceleration(closestPointFrame, desiredLinearAcceleration);
         desiredAcceleration.set(saturate(desiredAccelerationValue, maxAcceleration));
      }
      else
      {
         spatialAccelerationCommand.getSelectionMatrix().clearSelection();
         desiredAcceleration.set(0.0);
         closestPlanarRegion.set(-1);

         pointFeedbackCommand.getSpatialAccelerationCommand().set(spatialAccelerationCommand);
         pointFeedbackCommand.getGains().setProportionalGains(0.0);
         pointFeedbackCommand.getGains().setDerivativeGains(0.0);
         pointFeedbackCommand.getGains().setMaxFeedbackAndFeedbackRate(0.0, 0.0);
      }
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return pointFeedbackCommand;
   }

   public void handleCollisionManagerCommand(CollisionManagerCommand command)
   {
      this.expectedLength.set(command.getTest());
      int regions = command.getNumberOfPlanarRegions();
      numberOfPlanarSurfaces.set(regions);
      planarRegions.clear();

      for (int i = 0; i < regions; i++)
      {
         planarRegions.add();
         command.getPlanarRegionCommand(i).getPlanarRegion(planarRegions.getLast());
      }
   }
   
   private double computeDistanceFromPlanarRegion(PlanarRegion region, FrameVector3D distanceVector, FramePoint3D pointOnBody)
   {
      firstEndPoseInPlaneCoordinates.set(firstEndPose);
      otherEndPoseInPlaneCoordinates.set(otherEndPose);
      
      region.getTransformToWorld(planeToWorldTransform);
      planeFromWorldTransform.setAndInvert(planeToWorldTransform);

      firstEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);
      otherEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);


      if (firstEndPoseInPlaneCoordinates.getZ() * otherEndPoseInPlaneCoordinates.getZ() <= 0) //The two points are in two different semiplanes or at least one of them is on the plane
      {
         Point3D intersection = region.intersectWithLine(bodyLine);

         if (intersection != null)
         {
            distanceVector.set(region.getNormal());
            pointOnBody.set(intersection);
            return 0.0;
         }
      }
      
      double minDistance = -1.0;
      
      boolean firstProjectionIsInside = region.isPointInside(firstEndPoseInPlaneCoordinates.getX(),
                                                                     firstEndPoseInPlaneCoordinates.getY());
      
      boolean otherProjectionIsInside = region.isPointInside(otherEndPoseInPlaneCoordinates.getX(),
                                                                     otherEndPoseInPlaneCoordinates.getY());

      if (firstProjectionIsInside || otherProjectionIsInside)
      {
         boolean firstIsCloser = firstProjectionIsInside
               && (!otherProjectionIsInside || Math.abs(firstEndPoseInPlaneCoordinates.getZ()) < Math.abs(otherEndPoseInPlaneCoordinates.getZ()));

         if (firstIsCloser)
         {
            minDistance = Math.abs(firstEndPoseInPlaneCoordinates.getZ());
            pointOnBody.set(firstEndPose.getPosition());

            firstEndPoseInPlaneCoordinates.setZ(0.0);
            firstEndPoseInPlaneCoordinates.applyTransform(planeToWorldTransform);
            distanceVector.set(firstEndPoseInPlaneCoordinates.getX() - firstEndPose.getPosition().getX(),
                               firstEndPoseInPlaneCoordinates.getY() - firstEndPose.getPosition().getY(),
                               firstEndPoseInPlaneCoordinates.getZ() - firstEndPose.getPosition().getZ());
            firstEndPoseInPlaneCoordinates.set(firstEndPose);
            firstEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);

         }
         else
         {
            minDistance = Math.abs(otherEndPoseInPlaneCoordinates.getZ());
            pointOnBody.set(otherEndPose.getPosition());

            otherEndPoseInPlaneCoordinates.setZ(0.0);
            otherEndPoseInPlaneCoordinates.applyTransform(planeToWorldTransform);
            distanceVector.set(otherEndPoseInPlaneCoordinates.getX() - otherEndPose.getPosition().getX(),
                               otherEndPoseInPlaneCoordinates.getY() - otherEndPose.getPosition().getY(),
                               otherEndPoseInPlaneCoordinates.getZ() - otherEndPose.getPosition().getZ());
            otherEndPoseInPlaneCoordinates.set(otherEndPose);
            otherEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);
         }
      }

      FrameVector3D tempDistanceVector = new FrameVector3D();
      FramePoint3D tempMinPoint = new FramePoint3D();

      for (int polygon = 0; polygon < region.getNumberOfConvexPolygons(); ++polygon)
      {
         double distance = computeMinimumDistanceFromConvexHullEdges(region.getConvexPolygon(polygon), tempDistanceVector, tempMinPoint);

         if ((minDistance < -0.5) || (distance < minDistance))
         {
            minDistance = distance;
            distanceVector.set(tempDistanceVector);
            pointOnBody.set(tempMinPoint);
         }
      }

      distanceVector.normalize();

      return minDistance;
   }

   private double computeMinimumDistanceFromConvexHullEdges(ConvexPolygon2D polygon, FrameVector3D distanceVector, FramePoint3D pointOnBody)
   {
      double minDistance = -1.0;
      for (int v = 0; v < polygon.getNumberOfVertices(); ++v)
      {
         polygon.getEdge(v, edge);

         firstConcaveHullVertex.set(edge.getFirstEndpoint());
         secondConcaveHullVertex.set(edge.getSecondEndpoint());

         double distance = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(firstEndPoseInPlaneCoordinates.getPosition(),
                                                                                       otherEndPoseInPlaneCoordinates.getPosition(),
                                                                                       firstConcaveHullVertex,
                                                                                       secondConcaveHullVertex,
                                                                                       firstMinDistanceSegmentPointInPlaneCoordinates,
                                                                                       secondMinDistanceSegmentPointInPlaneCoordinates);

         if ((minDistance < -0.5) || (distance < minDistance))
         {
            firstMinDistanceSegmentPoint.set(firstMinDistanceSegmentPointInPlaneCoordinates);
            firstMinDistanceSegmentPoint.applyTransform(planeToWorldTransform);
            secondMinDistanceSegmentPoint.set(secondMinDistanceSegmentPointInPlaneCoordinates);
            secondMinDistanceSegmentPoint.applyTransform(planeToWorldTransform);

            minDistance = distance;
            distanceVector.set(secondMinDistanceSegmentPoint.getX() - firstMinDistanceSegmentPoint.getX(),
                               secondMinDistanceSegmentPoint.getY() - firstMinDistanceSegmentPoint.getY(),
                               secondMinDistanceSegmentPoint.getZ() - firstMinDistanceSegmentPoint.getZ());
            pointOnBody.set(firstMinDistanceSegmentPoint);
         }
      }

      return minDistance;
   }

   private double saturate(double input, double threshold)
   {
      return Math.max(-threshold, Math.min(threshold, input));
   }

}
