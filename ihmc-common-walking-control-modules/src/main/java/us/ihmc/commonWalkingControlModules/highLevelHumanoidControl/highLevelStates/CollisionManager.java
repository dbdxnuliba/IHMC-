package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CollisionManagerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollisionManager
{
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final RigidBodyBasics body;
   ReferenceFrame firstEndLinkFrame, otherEndLinkFrame;
   FramePose3D firstEndPose = new FramePose3D();
   FramePose3D otherEndPose = new FramePose3D();
   FramePose3D firstEndPoseInPlaneCoordinates = new FramePose3D();
   FramePose3D otherEndPoseInPlaneCoordinates = new FramePose3D();
   Line3D bodyLine = new Line3D();
   RigidBodyTransform planeToWorldTransform = new RigidBodyTransform();
   RigidBodyTransform planeFromWorldTransform = new RigidBodyTransform();
   Point3D firstMinDistanceSegmentPointInPlaneCoordinates = new Point3D();
   Point3D secondMinDistanceSegmentPointInPlaneCoordinates = new Point3D();
   Point3D firstMinDistanceSegmentPoint = new Point3D();
   Point3D secondMinDistanceSegmentPoint = new Point3D();
   Point3D firstConcaveHullVertex = new Point3D();
   Point3D secondConcaveHullVertex = new Point3D();
   Vector3D minDistanceVector = new Vector3D();
   Point3D closestPointOnBody = new Point3D();
   LineSegment2D edge = new LineSegment2D();


   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(100, PlanarRegion.class);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble bodyOriginX, bodyOriginY, bodyOriginZ;
   private final YoDouble bodyEndX, bodyEndY, bodyEndZ;
   private final YoDouble distanceX, distanceY, distanceZ;
   private final YoDouble closestBodyPointX, closestBodyPointY, closestBodyPointZ;
   private final YoInteger closestPlanarRegion;
   private final YoDouble measuredDistance, expectedLength, minimumDistanceValue;
   private final YoInteger numberOfPlanarSurfaces;

   public CollisionManager(ReferenceFrame firstEndLinkFrame, ReferenceFrame otherEndLinkFrame, RigidBodyBasics body,
                           RigidBodyBasics elevator, YoVariableRegistry parentRegistry)
   {
      spatialAccelerationCommand.set(/*
                                      * MultiBodySystemTools.getRootBody(body)
                                      */elevator, body); //Does this uses also the base acceleration?
      spatialAccelerationCommand.getWeightMatrix().setLinearWeights(10.0, 10.0, 10.0);
      this.firstEndLinkFrame = firstEndLinkFrame;
      this.otherEndLinkFrame = otherEndLinkFrame;
      this.body = body;
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

      double minDistance = 1.0;
      Vector3D distanceVector = new Vector3D();
      Point3D pointOnBody = new Point3D();

      for (int i = 0; i < planarRegions.size(); ++i)
      {
         double distance = computeDistanceFromPlanarRegion(planarRegions.get(i), distanceVector, pointOnBody);

         if ((minDistance < -0.5) || (distance < minDistance))
         {
            minDistance = distance;
            minDistanceVector.set(distanceVector);
            closestPointOnBody.set(pointOnBody);
            closestPlanarRegion.set(i);
         }
      }
      
      distanceX.set(minDistanceVector.getX());
      distanceY.set(minDistanceVector.getY());
      distanceZ.set(minDistanceVector.getZ());
      closestBodyPointX.set(closestPointOnBody.getX());
      closestBodyPointY.set(closestPointOnBody.getY());
      closestBodyPointZ.set(closestPointOnBody.getZ());
      minimumDistanceValue.set(minDistance);

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
   
   private double computeDistanceFromPlanarRegion(PlanarRegion region, Vector3D distanceVector, Point3D pointOnBody)
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
            pointOnBody.applyTransform(planeToWorldTransform);

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
               && (!otherProjectionIsInside || firstEndPoseInPlaneCoordinates.getZ() < otherEndPoseInPlaneCoordinates.getZ());

         if (firstIsCloser)
         {
            minDistance = Math.abs(firstEndPoseInPlaneCoordinates.getZ());
            pointOnBody.set(firstEndPose.getPosition());
            distanceVector.set(region.getNormal());
         }
         else
         {
            minDistance = Math.abs(otherEndPoseInPlaneCoordinates.getZ());
            pointOnBody.set(otherEndPose.getPosition());
            distanceVector.set(region.getNormal());
         }
      }

      Vector3D tempDistanceVector = new Vector3D();
      Point3D tempMinPoint = new Point3D();

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

      if (minDistance > 0.01)
      {
         distanceVector.normalize();
      }
      else
      {
         distanceVector.set(region.getNormal());
      }

      return minDistance;
   }

   private double computeMinimumDistanceFromConvexHullEdges(ConvexPolygon2D polygon, Vector3D distanceVector, Point3D pointOnBody)
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

}
