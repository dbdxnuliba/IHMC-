package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.CollisionAvoidanceManagerMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commonWalkingControlModules.configurations.CollisionAvoidanceParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CollisionAvoidanceManagerCommand;
import us.ihmc.humanoidRobotics.communication.packets.collisionAvoidance.CollisionAvoidanceMessageMode;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollisionAvoidanceManager
{
   private final PointFeedbackControlCommand pointFeedbackCommand = new PointFeedbackControlCommand();

   private final CollisionAvoidanceParameters param;
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
   private final Point3D firstConcaveHullVertexInPlaneCoordinates = new Point3D();
   private final Point3D secondConcaveHullVertexInPlaneCoordinates = new Point3D();
   private final FrameVector3D minDistanceVector = new FrameVector3D();
   private final FrameVector3D zAxis = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
   private final FramePoint3D closestPointOnBody = new FramePoint3D();
   private final AxisAngle worldZToDistanceVectorRotation = new AxisAngle();
   private final LineSegment2D edge = new LineSegment2D();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D zeroVector = new FrameVector3D();
   private final TransformReferenceFrame closestPointFrame;

   private final RecyclingArrayList<AtomicBoolean> considerOnlyEdgesVector = new RecyclingArrayList<>(100, AtomicBoolean.class);
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(100, PlanarRegion.class);

   private final YoVariableRegistry registry;

   private final YoDouble bodyOriginX, bodyOriginY, bodyOriginZ;
   private final YoDouble bodyEndX, bodyEndY, bodyEndZ;
   private final YoDouble distanceX, distanceY, distanceZ;
   private final YoDouble closestBodyPointX, closestBodyPointY, closestBodyPointZ;
   private final YoDouble desiredPositionX, desiredPositionY, desiredPositionZ;
   private final YoInteger closestPlanarRegion;
   private final YoDouble minimumDistanceValue;
   private final YoInteger numberOfPlanarSurfaces;
   private final YoGraphicVector distanceArrow, desiredPositionArrow;
   private final YoBoolean isActive;

   public CollisionAvoidanceManager(CollisionAvoidanceParameters parameters, ReferenceFrame firstEndLinkFrame, ReferenceFrame otherEndLinkFrame,
                                    RigidBodyBasics body, RigidBodyBasics elevator, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (!parameters.useCollisionAvoidance())
         throw new IllegalArgumentException("The CollisionAvoidanceManager is created but the useCollisionAvoidance parameter is set to false.");

      if (parameters.getActivationThreshold() > parameters.getDeactivationThreshold())
         throw new IllegalArgumentException("The activation threshold is supposed to be lower or equal to the deactivation threshold for the module to work properly.");

      param = parameters;
      pointFeedbackCommand.set(elevator, body);
      pointFeedbackCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      this.firstEndLinkFrame = ReferenceFrameTools.constructFrameWithUnchangingTranslationFromParent(firstEndLinkFrame.getName() + "_plus_offset",
                                                                                                     firstEndLinkFrame,
                                                                                                     parameters.getFirstFrameOffset());
      this.otherEndLinkFrame = ReferenceFrameTools.constructFrameWithUnchangingTranslationFromParent(otherEndLinkFrame.getName() + "_plus_offset",
                                                                                                     otherEndLinkFrame,
                                                                                                     parameters.getSecondFrameOffset());
      this.body = body;

      closestPointFrame = new TransformReferenceFrame(body.getName() + "_collisionFrame", body.getBodyFixedFrame());

      registry = new YoVariableRegistry(getClass().getSimpleName() + "_" + body.getName());
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

      numberOfPlanarSurfaces = new YoInteger("collision_" + body.getName() + "_numberOfPlanarSurfaces", registry);
      
      distanceArrow = new YoGraphicVector("ClosestCollisionVector_" + body.getName(),
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

      desiredPositionArrow = new YoGraphicVector("DesiredPositionToAvoidCollision_" + body.getName(),
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

      YoGraphicsList yoGraphicsList = new YoGraphicsList("CollisionAvoidanceManager_" + body.getName() + "_Graphics");
      yoGraphicsList.add(distanceArrow);
      yoGraphicsList.add(desiredPositionArrow);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      
   }

   private final FrameVector3D templPlaneDistanceVector = new FrameVector3D();
   private final FramePoint3D templPlaneClosestPointOnBody = new FramePoint3D();

   public void compute()
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

      bodyLine.set(firstEndPose.getPosition(), otherEndPose.getPosition());

      double minDistance = param.getDeactivationThreshold();

      for (int i = 0; i < planarRegions.size(); ++i)
      {
         double distance = computeDistanceFromPlanarRegion(planarRegions.get(i),
                                                           considerOnlyEdgesVector.get(i).get(),
                                                           templPlaneDistanceVector,
                                                           templPlaneClosestPointOnBody);

         if ((distance >= 0) && (distance < minDistance))
         {
            minDistance = distance;
            minDistanceVector.set(templPlaneDistanceVector);
            closestPointOnBody.set(templPlaneClosestPointOnBody);
            closestPlanarRegion.set(i);
         }
      }
      
      distanceX.set(minDistanceVector.getX() * minDistance);
      distanceY.set(minDistanceVector.getY() * minDistance);
      distanceZ.set(minDistanceVector.getZ() * minDistance);

      if (minDistanceVector.getZ() > param.getMaximumVerticalDistanceComponent())
      {
         minDistanceVector.setZ(param.getMaximumVerticalDistanceComponent());
         minDistanceVector.normalize();
      }
      
      closestBodyPointX.set(closestPointOnBody.getX());
      closestBodyPointY.set(closestPointOnBody.getY());
      closestBodyPointZ.set(closestPointOnBody.getZ());
      minimumDistanceValue.set(minDistance);

      setupCommands(minDistance, minDistanceVector);
   }

   private final RigidBodyTransform body_H_closestPointAsRBT = new RigidBodyTransform();
   private final WeightMatrix3D weights = new WeightMatrix3D();
   private final SelectionMatrix3D selection = new SelectionMatrix3D();

   private void setupCommands(double minDistance, FrameVector3D distanceVector)
   {
      if (minDistance >= 0.0
            && (((minDistance < param.getActivationThreshold()) && !isActive.getBooleanValue())
                  || ((minDistance < param.getDeactivationThreshold()) && isActive.getBooleanValue())))
      {
         updateClosestPointFrame();

         selection.setSelectionFrame(closestPointFrame);
         selection.setAxisSelection(false, false, true);
         pointFeedbackCommand.setSelectionMatrix(selection);

         weights.setWeightFrame(closestPointFrame);
         weights.setWeights(0.0, 0.0, param.getWeight());
         pointFeedbackCommand.setWeightMatrix(weights);
         pointFeedbackCommand.setBodyFixedPointToControl(body_H_closestPoint.getPosition());
         pointFeedbackCommand.getGains().setProportionalGains(param.getProportionalGain());
         pointFeedbackCommand.getGains().setDerivativeGains(param.getDerivativeGain());
         pointFeedbackCommand.getGains().setIntegralGains(0.0, 0.0);
         pointFeedbackCommand.getGains().setMaxFeedbackAndFeedbackRate(param.getMaxFeedback(), param.getMaxFeedbackVariation());

         desiredPosition.setToZero(ReferenceFrame.getWorldFrame());
         desiredPosition.set(closestPointOnBody.getX() - (param.getActivationThreshold() - minDistance) * distanceVector.getX(),
                             closestPointOnBody.getY() - (param.getActivationThreshold() - minDistance) * distanceVector.getY(),
                             closestPointOnBody.getZ() - (param.getActivationThreshold() - minDistance) * distanceVector.getZ());

         pointFeedbackCommand.setInverseDynamics(desiredPosition, zeroVector, zeroVector);
         
         desiredPositionX.set(-(param.getActivationThreshold() - minDistance) * distanceVector.getX());
         desiredPositionY.set(-(param.getActivationThreshold() - minDistance) * distanceVector.getY());
         desiredPositionZ.set(-(param.getActivationThreshold() - minDistance) * distanceVector.getZ());

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

   private void updateClosestPointFrame()
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

      closestPointFrame.setTransformAndUpdate(body_H_closestPointAsRBT);
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
   
   private final FrameVector3D tempEdgeDistanceVector = new FrameVector3D();
   private final FramePoint3D tempEdgeBodyClosestPoint = new FramePoint3D();
   private final Point3D shinIntersectionWithRegion = new Point3D();

   private double computeDistanceFromPlanarRegion(PlanarRegion region, boolean considerOnlyEdges, FrameVector3D distanceVectorToPack, FramePoint3D pointOnBodyToPack)
   {
      firstEndPoseInPlaneCoordinates.set(firstEndPose);
      otherEndPoseInPlaneCoordinates.set(otherEndPose);
      
      region.getTransformToWorld(planeToWorldTransform);
      planeFromWorldTransform.setAndInvert(planeToWorldTransform);

      firstEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);
      otherEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);

      boolean probablyIntersecting = firstEndPoseInPlaneCoordinates.getZ() * otherEndPoseInPlaneCoordinates.getZ() <= 0; //The two points are in two different semiplanes or at least one of them is on the plane

      if (!probablyIntersecting
            && Math.min(Math.abs(firstEndPoseInPlaneCoordinates.getZ()), Math.abs(otherEndPoseInPlaneCoordinates.getZ())) > param.getDeactivationThreshold()) //The plane is too distant
      {
         return -1.0;
      }

      double minDistance = -1.0;

      if (!considerOnlyEdges)
      {
         if (probablyIntersecting)
         {
            boolean intersecting = intersectRegionWithLine(region, bodyLine, shinIntersectionWithRegion);

            if (intersecting)
            {
               distanceVectorToPack.set(region.getNormal());
               pointOnBodyToPack.set(shinIntersectionWithRegion);
               return 0.0;
            }
         }

         boolean firstProjectionIsInside = region.isPointInside(firstEndPoseInPlaneCoordinates.getX(), firstEndPoseInPlaneCoordinates.getY());

         boolean otherProjectionIsInside = region.isPointInside(otherEndPoseInPlaneCoordinates.getX(), otherEndPoseInPlaneCoordinates.getY());

         if (firstProjectionIsInside || otherProjectionIsInside)
         {
            boolean firstIsCloser = firstProjectionIsInside
                  && (!otherProjectionIsInside || Math.abs(firstEndPoseInPlaneCoordinates.getZ()) < Math.abs(otherEndPoseInPlaneCoordinates.getZ()));

            if (firstIsCloser)
            {
               minDistance = Math.abs(firstEndPoseInPlaneCoordinates.getZ());
               pointOnBodyToPack.set(firstEndPose.getPosition());

               firstEndPoseInPlaneCoordinates.setZ(0.0);
               firstEndPoseInPlaneCoordinates.applyTransform(planeToWorldTransform);
               distanceVectorToPack.set(firstEndPoseInPlaneCoordinates.getX() - firstEndPose.getPosition().getX(),
                                  firstEndPoseInPlaneCoordinates.getY() - firstEndPose.getPosition().getY(),
                                  firstEndPoseInPlaneCoordinates.getZ() - firstEndPose.getPosition().getZ());
               firstEndPoseInPlaneCoordinates.set(firstEndPose);
               firstEndPoseInPlaneCoordinates.applyTransform(planeFromWorldTransform);

            }
            else
            {
               minDistance = Math.abs(otherEndPoseInPlaneCoordinates.getZ());
               pointOnBodyToPack.set(otherEndPose.getPosition());

               otherEndPoseInPlaneCoordinates.setZ(0.0);
               otherEndPoseInPlaneCoordinates.applyTransform(planeToWorldTransform);
               distanceVectorToPack.set(otherEndPoseInPlaneCoordinates.getX() - otherEndPose.getPosition().getX(),
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

         if ((minDistance < 0) || ((distance >= 0) && (distance < minDistance)))
         {
            minDistance = distance;
            distanceVectorToPack.set(tempEdgeDistanceVector);
            pointOnBodyToPack.set(tempEdgeBodyClosestPoint);
         }
      }

      distanceVectorToPack.normalize();

      return minDistance;
   }

   private double computeMinimumDistanceFromConvexHullEdges(ConvexPolygon2D polygon, FrameVector3D distanceVectorToPack, FramePoint3D pointOnBodyToPack)
   {
      double minDistance = -1.0;
      for (int v = 0; v < polygon.getNumberOfVertices(); ++v)
      {
         polygon.getEdge(v, edge);

         firstConcaveHullVertexInPlaneCoordinates.set(edge.getFirstEndpoint());
         secondConcaveHullVertexInPlaneCoordinates.set(edge.getSecondEndpoint());

         firstConcaveHullVertex.set(firstConcaveHullVertexInPlaneCoordinates);
         firstConcaveHullVertex.applyTransform(planeToWorldTransform);
         secondConcaveHullVertex.set(secondConcaveHullVertexInPlaneCoordinates);
         secondConcaveHullVertex.applyTransform(planeToWorldTransform);

         if (!param.ignoreEdgesAtLowerHeight()
               || (Math.max(firstConcaveHullVertex.getZ(), secondConcaveHullVertex.getZ()) > Math.min(firstEndPose.getZ(), otherEndPose.getZ())))
         {
            double distance = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(firstEndPoseInPlaneCoordinates.getPosition(),
                                                                                          otherEndPoseInPlaneCoordinates.getPosition(),
                                                                                          firstConcaveHullVertexInPlaneCoordinates,
                                                                                          secondConcaveHullVertexInPlaneCoordinates,
                                                                                          firstMinDistanceSegmentPointInPlaneCoordinates,
                                                                                          secondMinDistanceSegmentPointInPlaneCoordinates);

            if ((minDistance < 0) || (distance < minDistance))
            {
               firstMinDistanceSegmentPoint.set(firstMinDistanceSegmentPointInPlaneCoordinates);
               firstMinDistanceSegmentPoint.applyTransform(planeToWorldTransform);
               secondMinDistanceSegmentPoint.set(secondMinDistanceSegmentPointInPlaneCoordinates);
               secondMinDistanceSegmentPoint.applyTransform(planeToWorldTransform);

               if (!param.ignoreEdgesAtLowerHeight() || (secondMinDistanceSegmentPoint.getZ() > Math.min(firstEndPose.getZ(), otherEndPose.getZ())))
               {
                  minDistance = distance;
                  distanceVectorToPack.set(secondMinDistanceSegmentPoint.getX() - firstMinDistanceSegmentPoint.getX(),
                                           secondMinDistanceSegmentPoint.getY() - firstMinDistanceSegmentPoint.getY(),
                                           secondMinDistanceSegmentPoint.getZ() - firstMinDistanceSegmentPoint.getZ());
                  pointOnBodyToPack.set(firstMinDistanceSegmentPoint);
               }

            }
         }
      }

      return minDistance;
   }

   private final Point3D genericPointOnPlane = new Point3D();
   private final Point3D pointOnLineInLocal = new Point3D();
   private final Vector3D directionOfLineInLocal = new Vector3D();
   private final Vector3DReadOnly genericPlaneNormal = new Vector3D(0.0, 0.0, 1.0);

   private boolean intersectRegionWithLine(PlanarRegion region, Line3D projectionLineInWorld, Point3D intersectionWithPlaneToPack)
   {
      genericPointOnPlane.set(region.getConvexPolygon(0).getVertex(0));

      pointOnLineInLocal.set(projectionLineInWorld.getPoint());
      directionOfLineInLocal.set(projectionLineInWorld.getDirection());

      region.transformFromWorldToLocal(pointOnLineInLocal);
      region.transformFromWorldToLocal(directionOfLineInLocal);

      boolean success = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(genericPointOnPlane,
                                                                                genericPlaneNormal,
                                                                                pointOnLineInLocal,
                                                                                directionOfLineInLocal,
                                                                                intersectionWithPlaneToPack);
      if (!success) // line was parallel to plane
      {
         return false;
      }

      if (region.isPointInside(intersectionWithPlaneToPack.getX(), intersectionWithPlaneToPack.getY()))
      {
         region.transformFromLocalToWorld(intersectionWithPlaneToPack);
         return true;
      }

      return false; // line does not intersect
   }

   /**
    * Computes the vertical planar regions which have as top edge one edge of the horizontal planar region, while the bottom edge is the projection on
    * the ground of the top edge. By assumption, the ground is at 0. If not, it should be enough to modify the computation of the newPosition. In fact
    * it is assumed that the origin of the plane frame is located in the centroid of the 4 edges.
    * @param horizontalRegion Region from which the vertical regions are generated
    * @param messageToModify Message to which add the planar regions
    * @return true if successful
    */
   public static boolean addVerticalRegionsFromHorizontalRegions(PlanarRegion horizontalRegion, CollisionAvoidanceManagerMessage messageToModify)
   {
      if (horizontalRegion == null)
      {
         LogTools.error("The input horizontal region is null.");
         return false;
      }
      if (messageToModify == null)
      {
         LogTools.error("The message to modify is null.");
         return false;
      }

      if (Math.abs(horizontalRegion.getNormal().getZ()) < 0.1)
      {
         LogTools.error("The input horizontal region seems to be vertical.");
         return false;
      }

      final LineSegment2D edge = new LineSegment2D();
      final LineSegment3D edgeInWorld = new LineSegment3D();
      final Point3D firstEndPointInWorld = new Point3D();
      final Point3D secondEndPointInWorld = new Point3D();
      final ConvexPolygon2D newPolygon = new ConvexPolygon2D();
      final Vector3D gravityDirection = new Vector3D(0, 0, 1.0);
      final Vector3D planeNormal = new Vector3D();
      final Vector3D newPosition = new Vector3D();
      final Vector3D originsDistance = new Vector3D();
      final Vector3D planeYDirection = new Vector3D();
      final RotationMatrix planeRotation = new RotationMatrix();
      final RigidBodyTransform planeTransform = new RigidBodyTransform();
      final LineSegment3D edgeInNewPlane = new LineSegment3D();
      final PlanarRegion newRegion = new PlanarRegion();
      final RecyclingArrayList<ConvexPolygon2D> newList = new RecyclingArrayList<>(1, ConvexPolygon2D.class);

      for (int p = 0; p < horizontalRegion.getNumberOfConvexPolygons(); ++p)
      {
         ConvexPolygon2D polygon = horizontalRegion.getConvexPolygon(p);
         newPolygon.clear();

         for (int v = 0; v < polygon.getNumberOfVertices(); ++v)
         {
            polygon.getEdge(v, edge);
            firstEndPointInWorld.set(edge.getFirstEndpoint());
            secondEndPointInWorld.set(edge.getSecondEndpoint());
            edgeInWorld.set(firstEndPointInWorld, secondEndPointInWorld);

            horizontalRegion.transformFromLocalToWorld(edgeInWorld); //Edge in world coordinates
            Vector3DBasics edgeDirection = edgeInWorld.getDirection(true); //Get its normalized direction

            planeNormal.set(gravityDirection);
            planeNormal.cross(edgeDirection); //The plane is formed by the edge direction and by the gravity direction
            planeNormal.normalize();

            newPosition.set((edgeInWorld.getFirstEndpointX() + edgeInWorld.getSecondEndpointX()) * 0.5,
                            (edgeInWorld.getFirstEndpointY() + edgeInWorld.getSecondEndpointY()) * 0.5,
                            (edgeInWorld.getFirstEndpointZ() + edgeInWorld.getSecondEndpointZ()) * 0.25); //The centroid of the plane assuming two points having 0 as coordinate

            originsDistance.setToZero();
            horizontalRegion.transformFromLocalToWorld(originsDistance); //Horizontal plane origin in world coordinates
            originsDistance.sub(newPosition); //Distance vector between the two origins in world coordinates

            if (originsDistance.dot(planeNormal) < 0) // I want the normal to be pointing inward
            {
               planeNormal.scale(-1);
            }

            planeYDirection.set(planeNormal);
            planeYDirection.cross(gravityDirection); //Find the y direction. The normal has been computed before, while the x is parallel to gravity
            planeYDirection.normalize();

            planeRotation.setColumns(gravityDirection, planeYDirection, planeNormal);
            planeRotation.normalize();

            planeTransform.setTranslation(newPosition);
            planeTransform.setRotation(planeRotation);

            edgeInNewPlane.set(edgeInWorld);
            edgeInNewPlane.applyInverseTransform(planeTransform);

            newPolygon.addVertex(edgeInNewPlane.getFirstEndpointX(), edgeInNewPlane.getFirstEndpointY());
            newPolygon.addVertex(-edgeInNewPlane.getFirstEndpointX(), edgeInNewPlane.getFirstEndpointY());
            newPolygon.addVertex(-edgeInNewPlane.getSecondEndpointX(), edgeInNewPlane.getSecondEndpointX());
            newPolygon.addVertex(edgeInNewPlane.getSecondEndpointX(), edgeInNewPlane.getSecondEndpointX());
            newPolygon.update();

            newList.clear();
            newList.add().set(newPolygon);
            newRegion.set(planeTransform, newList);
            PlanarRegionMessage newPlanarRegionMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(newRegion);
            messageToModify.getPlanarRegionsList().add().set(newPlanarRegionMessage);
         }
      }

      return true;
   }

}
