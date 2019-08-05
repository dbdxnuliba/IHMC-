package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
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
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CollisionAvoidanceManagerCommand;
import us.ihmc.humanoidRobotics.communication.packets.collisionAvoidance.CollisionAvoidanceMessageMode;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollisionAvoidanceManager
{
   private final PointFeedbackControlCommand pointFeedbackCommand = new PointFeedbackControlCommand();

   private final RigidBodyBasics body;
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

   private final RecyclingArrayList<AtomicBoolean> considerOnlyEdgesVector = new RecyclingArrayList<>(100, AtomicBoolean.class);
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(100, PlanarRegion.class);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble bodyOriginX, bodyOriginY, bodyOriginZ;
   private final YoDouble bodyEndX, bodyEndY, bodyEndZ;
   private final YoDouble distanceX, distanceY, distanceZ;
   private final YoDouble closestBodyPointX, closestBodyPointY, closestBodyPointZ;
   private final YoDouble desiredPositionX, desiredPositionY, desiredPositionZ;
   private final YoInteger closestPlanarRegion;
   private final YoDouble measuredDistance, minimumDistanceValue;
   private final YoInteger numberOfPlanarSurfaces;
   private final YoGraphicVector distanceArrow, desiredPositionArrow;
   private final YoBoolean isActive;

   public CollisionAvoidanceManager(ReferenceFrame firstEndLinkFrame, ReferenceFrame otherEndLinkFrame, RigidBodyBasics body,
                           RigidBodyBasics elevator, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      pointFeedbackCommand.set(elevator, body);
      pointFeedbackCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

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
      desiredPositionX = new YoDouble("collision_" + body.getName() + "_desiredPositionX", registry);
      desiredPositionY = new YoDouble("collision_" + body.getName() + "_desiredPositionY", registry);
      desiredPositionZ = new YoDouble("collision_" + body.getName() + "_desiredPositionZ", registry);
      minimumDistanceValue = new YoDouble("collision_" + body.getName() + "_minDistanceValue", registry);
      closestPlanarRegion = new YoInteger("collision_" + body.getName() + "_closestRegion", registry);
      isActive = new YoBoolean("collision_" + body.getName() + "_active", registry);

      measuredDistance = new YoDouble("collision_" + body.getName() + "measuredLenght", registry);
      
      numberOfPlanarSurfaces = new YoInteger("collision_numberOfPlanarSurfaces", registry);
      
      distanceArrow = new YoGraphicVector("ClosestCollisionVector",
                                          closestBodyPointX,
                                          closestBodyPointY,
                                          closestBodyPointZ,
                                          distanceX,
                                          distanceY,
                                          distanceZ,
                                          1.0,
                                          YoAppearance.BlackMetalMaterial(),
                                          true);
      distanceArrow.setLineRadiusWhenOneMeterLong(0.03);

      desiredPositionArrow = new YoGraphicVector("DesiredPositionToAvoidCollision",
                                                 closestBodyPointX,
                                                 closestBodyPointY,
                                                 closestBodyPointZ,
                                                 desiredPositionX,
                                                 desiredPositionY,
                                                 desiredPositionZ,
                                                 1.0,
                                                 YoAppearance.DarkRed(),
                                                 true);
      desiredPositionArrow.setLineRadiusWhenOneMeterLong(0.03);

      YoGraphicsList yoGraphicsList = new YoGraphicsList("CollisionAvoidanceManagerGraphics");
      yoGraphicsList.add(distanceArrow);
      yoGraphicsList.add(desiredPositionArrow);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      
   }

   private FrameVector3D templPlaneDistanceVector = new FrameVector3D();
   private FramePoint3D templPlaneClosestPointOnBody = new FramePoint3D();

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

      double activationThreshold = 0.1;
      double deactivationThreshold = 1.2 * activationThreshold;

      double minDistance = deactivationThreshold;

      for (int i = 0; i < planarRegions.size(); ++i)
      {
         double distance = computeDistanceFromPlanarRegion(planarRegions.get(i),
                                                           considerOnlyEdgesVector.get(i).get(),
                                                           templPlaneDistanceVector,
                                                           templPlaneClosestPointOnBody);

         if (distance < minDistance)
         {
            minDistance = distance;
            minDistanceVector.set(templPlaneDistanceVector);
            closestPointOnBody.set(templPlaneClosestPointOnBody);
            closestPlanarRegion.set(i);
         }
      }
      
      double maxZComponent = -0.1;

      if (minDistanceVector.getZ() > maxZComponent)
      {
         minDistanceVector.setZ(maxZComponent);
         minDistanceVector.normalize();
      }
      
      distanceX.set(minDistanceVector.getX() * minDistance);
      distanceY.set(minDistanceVector.getY() * minDistance);
      distanceZ.set(minDistanceVector.getZ() * minDistance);
      closestBodyPointX.set(closestPointOnBody.getX());
      closestBodyPointY.set(closestPointOnBody.getY());
      closestBodyPointZ.set(closestPointOnBody.getZ());
      minimumDistanceValue.set(minDistance);

      setupCommands(activationThreshold, deactivationThreshold, minDistance, minDistanceVector);
   }

   private RigidBodyTransform body_H_closestPointAsRBT = new RigidBodyTransform();
   private WeightMatrix3D weights = new WeightMatrix3D();
   private SelectionMatrix3D selection = new SelectionMatrix3D();

   private void setupCommands(double activationThreshold, double deactivationThreshold, double minDistance, FrameVector3D distanceVector)
   {
      if (minDistance >= 0.0
            && (((minDistance < activationThreshold) && !isActive.getBooleanValue()) || ((minDistance < deactivationThreshold) && isActive.getBooleanValue())))
      {
         ReferenceFrame closestPointFrame = computeClosestPointFrame();

         double accelerationGain = 500.0;
         double maxFeedback = 250.0;

         selection.setSelectionFrame(closestPointFrame);
         selection.setAxisSelection(false, false, true);
         pointFeedbackCommand.setSelectionMatrix(selection);

         weights.setWeightFrame(closestPointFrame);
         weights.setWeights(0.0, 0.0, 10.0);
         pointFeedbackCommand.setWeightMatrix(weights);
         pointFeedbackCommand.setBodyFixedPointToControl(body_H_closestPoint.getPosition());
         pointFeedbackCommand.getGains().setProportionalGains(accelerationGain);
         pointFeedbackCommand.getGains().setDerivativeGains(5.0 * Math.sqrt(accelerationGain));
         pointFeedbackCommand.getGains().setIntegralGains(0.0, 0.0);
         pointFeedbackCommand.getGains().setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedback * 10);

         desiredPosition.setToZero(ReferenceFrame.getWorldFrame());
         desiredPosition.set(closestPointOnBody.getX() - (activationThreshold - minDistance) * distanceVector.getX(),
                             closestPointOnBody.getY() - (activationThreshold - minDistance) * distanceVector.getY(),
                             closestPointOnBody.getZ() - (activationThreshold - minDistance) * distanceVector.getZ());

         pointFeedbackCommand.setInverseDynamics(desiredPosition, zeroVector, zeroVector);
         
         desiredPositionX.set(-(activationThreshold - minDistance) * distanceVector.getX());
         desiredPositionY.set(-(activationThreshold - minDistance) * distanceVector.getY());
         desiredPositionZ.set(-(activationThreshold - minDistance) * distanceVector.getZ());

         distanceArrow.showGraphicObject();
         desiredPositionArrow.showGraphicObject();
         isActive.set(true);

      }
      else
      {
         distanceArrow.hide();
         desiredPositionArrow.hide();
         closestPlanarRegion.set(-1);
         isActive.set(false);

         selection.clearSelection();
         pointFeedbackCommand.setSelectionMatrix(selection);
         pointFeedbackCommand.getGains().setProportionalGains(0.0);
         pointFeedbackCommand.getGains().setDerivativeGains(0.0);
         pointFeedbackCommand.getGains().setMaxFeedbackAndFeedbackRate(0.0, 0.0);
      }
   }

   private ReferenceFrame computeClosestPointFrame()
   {
      EuclidFrameTools.orientation3DFromFirstToSecondVector3D(zAxis, minDistanceVector, worldZToDistanceVectorRotation);

      body_H_closestPoint.setToZero(ReferenceFrame.getWorldFrame());
      body_H_closestPoint.setPosition(closestPointOnBody);
      body_H_closestPoint.setOrientation(worldZToDistanceVectorRotation); //This frame is located on the point of
                                                                       //the body at the minimum distance from the closest obstacle 
                                                                       //and oriented such that the Z-axis is aligned 
                                                                       //with the distance direction
      body_H_closestPoint.changeFrame(body.getBodyFixedFrame()); //Express the pose in body coordinates

      body_H_closestPointAsRBT.set(body_H_closestPoint.getOrientation(), body_H_closestPoint.getPosition());

      ReferenceFrame closestPointFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(body.getName() + "_collisionFrame",
                                                                                                             body.getBodyFixedFrame(),
                                                                                                             body_H_closestPointAsRBT); //This is allocating memory
      return closestPointFrame;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return pointFeedbackCommand;
   }

   public void handleCollisionAvoidanceManagerCommand(CollisionAvoidanceManagerCommand command)
   {
      int newRegions = command.getNumberOfPlanarRegions();

      if (command.getMode() == CollisionAvoidanceMessageMode.OVERRIDE)
      {
         planarRegions.clear();
         considerOnlyEdgesVector.clear();
         numberOfPlanarSurfaces.set(0);
      }
      numberOfPlanarSurfaces.set(newRegions + numberOfPlanarSurfaces.getIntegerValue());

      for (int i = 0; i < newRegions; i++)
      {
         PlanarRegion newRegion = planarRegions.add();
         command.getPlanarRegionCommand(i).getPlanarRegion(newRegion);
         considerOnlyEdgesVector.add().set(command.considerOnlyEdges());
      }
   }
   
   private FrameVector3D tempEdgeDistanceVector = new FrameVector3D();
   private FramePoint3D tempEdgeBodyClosestPoint = new FramePoint3D();

   private double computeDistanceFromPlanarRegion(PlanarRegion region, boolean considerOnlyEdges, FrameVector3D distanceVector, FramePoint3D pointOnBody)
   {
      firstEndPoseInPlaneCoordinates.set(firstEndPose);
      otherEndPoseInPlaneCoordinates.set(otherEndPose);
      
      region.getTransformToWorld(planeToWorldTransform);
      planeFromWorldTransform.setAndInvert(planeToWorldTransform);

      firstEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);
      otherEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);


      if (firstEndPoseInPlaneCoordinates.getZ() * otherEndPoseInPlaneCoordinates.getZ() <= 0) //The two points are in two different semiplanes or at least one of them is on the plane
      {
         Point3D intersection = region.intersectWithLine(bodyLine); //This is allocating memory

         if (intersection != null)
         {
            distanceVector.set(region.getNormal());
            pointOnBody.set(intersection);
            return 0.0;
         }
      }
      
      double minDistance = -1.0;
      
      if (!considerOnlyEdges)
      {
         boolean firstProjectionIsInside = region.isPointInside(firstEndPoseInPlaneCoordinates.getX(), firstEndPoseInPlaneCoordinates.getY());

         boolean otherProjectionIsInside = region.isPointInside(otherEndPoseInPlaneCoordinates.getX(), otherEndPoseInPlaneCoordinates.getY());

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
      }

      for (int polygon = 0; polygon < region.getNumberOfConvexPolygons(); ++polygon)
      {
         double distance = computeMinimumDistanceFromConvexHullEdges(region.getConvexPolygon(polygon), tempEdgeDistanceVector, tempEdgeBodyClosestPoint);

         if ((minDistance < -0.5) || (distance < minDistance))
         {
            minDistance = distance;
            distanceVector.set(tempEdgeDistanceVector);
            pointOnBody.set(tempEdgeBodyClosestPoint);
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

}
