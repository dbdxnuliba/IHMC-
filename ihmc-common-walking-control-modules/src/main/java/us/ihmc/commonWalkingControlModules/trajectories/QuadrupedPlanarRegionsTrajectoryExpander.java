package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.algorithms.SphereWithConvexPolygonIntersector;
import us.ihmc.robotics.geometry.shapes.FrameSphere3d;
import us.ihmc.robotics.math.YoCounter;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.*;

public class QuadrupedPlanarRegionsTrajectoryExpander
{
   private static final double minSwingHeight = 0.04;
   private static final double maxSwingHeight = 0.3;
   private static final double defaultCollisionSphereRadius = 0.05;
   private static final double defaultSwingHeight = 0.05;
   private static final double touchdownVelocityZ = -0.1;

   private static final double stepHeightChangeForObstacleAvoidance = 0.04;

   private static final int numberOfPointsToCheck = 100;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double ignoreDistanceFromFloor = 0.02;
   private static final double[] twoWaypoingSwingProportions = TwoWaypointSwingGenerator.getDefaultWaypointProportions();
   private static final double oneWaypointSwingProportion = 0.5;

   private final TwoWaypointSwingGenerator twoWaypointSwingGenerator;
   private final OneWaypointSwingGenerator oneWaypointSwingGenerator;

   private final YoInteger numberOfCheckpoints;
   private final YoInteger numberOfWaypoints;
   private final YoCounter numberOfTriesCounter;
   private final YoDouble minimumClearance;
   private final YoDouble incrementalAdjustmentDistance;
   private final YoDouble maximumAdjustmentDistance;
   private final YoDouble stepDurationToCheckWith;
   private final YoEnum<SwingOverPlanarRegionsTrajectoryCollisionType> mostSevereCollisionType;
   private final YoEnum<SwingOverPlanarRegionsTrajectoryExpansionStatus> status;

   private final YoFramePoint3D trajectoryPosition;
   private final RecyclingArrayList<FramePoint3D> originalWaypoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> adjustedWaypoints = new RecyclingArrayList<>(FramePoint3D::new);

   private double collisionSphereRadius;

   private final SphereWithConvexPolygonIntersector sphereWithConvexPolygonIntersector;
   private final Map<SwingOverPlanarRegionsTrajectoryCollisionType, FramePoint3D> closestPolygonPointMap;
   private final FrameSphere3d footCollisionSphere;
   private final FrameConvexPolygon2D framePlanarRegion;
   private final TransformReferenceFrame planarRegionReferenceFrame;
   private final FramePoint3D midGroundPoint;
   private final Vector3D waypointAdjustmentVector;
   private final Plane3D swingTrajectoryPlane;
   private final Plane3D swingFloorPlane;
   private final Plane3D frontOfCollisionPlaneFacingEnd;
   private final Plane3D backOfCollisionPlaneFacingStart;
   private final AxisAngle axisAngle;
   private final RigidBodyTransform rigidBodyTransform;

   private final Point3D tempPointOnPlane = new Point3D();
   private final Vector3D tempPlaneNormal = new Vector3D();

   // Boilerplate variables
   private final FrameVector3D initialVelocity;
   private final FrameVector3D touchdownVelocity;
   private final FramePoint3D swingStartPosition;
   private final FramePoint3D swingEndPosition;

   // Anti-garbage variables
   private final RigidBodyTransform planarRegionTransform;

   // Visualization
   private Optional<Updatable> visualizer;

   public enum SwingOverPlanarRegionsTrajectoryCollisionType
   {
      NO_INTERSECTION, INTERSECTION_BUT_BELOW_IGNORE_PLANE, INTERSECTION_BUT_OUTSIDE_TRAJECTORY, CRITICAL_INTERSECTION,
   }

   public enum SwingOverPlanarRegionsTrajectoryExpansionStatus
   {
      INITIALIZED, FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE, SEARCHING_FOR_SOLUTION, SOLUTION_FOUND,
   }

   public QuadrupedPlanarRegionsTrajectoryExpander(YoVariableRegistry parentRegistry,
                                                   YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = "trajectoryExpander";
      twoWaypointSwingGenerator = new TwoWaypointSwingGenerator(namePrefix + "Two", minSwingHeight, maxSwingHeight, defaultSwingHeight, parentRegistry,
                                                                graphicsListRegistry);
      oneWaypointSwingGenerator = new OneWaypointSwingGenerator(namePrefix + "One", minSwingHeight, maxSwingHeight, defaultSwingHeight, parentRegistry,
                                                                graphicsListRegistry);

      collisionSphereRadius = defaultCollisionSphereRadius;

      numberOfCheckpoints = new YoInteger(namePrefix + "NumberOfCheckpoints", parentRegistry);
      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", parentRegistry);
      numberOfTriesCounter = new YoCounter(namePrefix + "NumberOfTriesCounter", parentRegistry);
      minimumClearance = new YoDouble(namePrefix + "MinimumClearance", parentRegistry);
      incrementalAdjustmentDistance = new YoDouble(namePrefix + "IncrementalAdjustmentDistance", parentRegistry);
      maximumAdjustmentDistance = new YoDouble(namePrefix + "MaximumAdjustmentDistance", parentRegistry);
      status = new YoEnum<>(namePrefix + "Status", parentRegistry, SwingOverPlanarRegionsTrajectoryExpansionStatus.class);
      mostSevereCollisionType = new YoEnum<>(namePrefix + "CollisionType", parentRegistry, SwingOverPlanarRegionsTrajectoryCollisionType.class);
      stepDurationToCheckWith = new YoDouble("stepDurationToCheckWith", parentRegistry);
      stepDurationToCheckWith.set(1.0);

      trajectoryPosition = new YoFramePoint3D(namePrefix + "TrajectoryPosition", worldFrame, parentRegistry);

      sphereWithConvexPolygonIntersector = new SphereWithConvexPolygonIntersector();
      closestPolygonPointMap = new HashMap<>();
      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         closestPolygonPointMap.put(swingOverPlanarRegionsTrajectoryCollisionType, new FramePoint3D());
      }
      footCollisionSphere = new FrameSphere3d();
      footCollisionSphere.setRadius(defaultCollisionSphereRadius);
      framePlanarRegion = new FrameConvexPolygon2D();
      planarRegionReferenceFrame = new TransformReferenceFrame("planarRegionReferenceFrame", worldFrame);
      midGroundPoint = new FramePoint3D();
      waypointAdjustmentVector = new Vector3D();
      swingTrajectoryPlane = new Plane3D();
      swingFloorPlane = new Plane3D();
      frontOfCollisionPlaneFacingEnd = new Plane3D();
      backOfCollisionPlaneFacingStart = new Plane3D();
      axisAngle = new AxisAngle();
      rigidBodyTransform = new RigidBodyTransform();

      initialVelocity = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);
      touchdownVelocity = new FrameVector3D(worldFrame, 0.0, 0.0, touchdownVelocityZ);
      swingStartPosition = new FramePoint3D();
      swingEndPosition = new FramePoint3D();

      planarRegionTransform = new RigidBodyTransform();

      visualizer = Optional.empty();

      // Set default values
      numberOfCheckpoints.set(numberOfPointsToCheck);
      numberOfTriesCounter.setMaxCount(50);
      minimumClearance.set(0.04);
      incrementalAdjustmentDistance.set(0.03);
   }

   public double expandTrajectoryOverPlanarRegions(FramePoint3DReadOnly swingStartPosition, FramePoint3DReadOnly swingEndPosition, double swingHeight,
                                                   PlanarRegionsList planarRegionsList, TrajectoryType trajectoryType)
   {
      trajectoryType = getTrajectoryType(trajectoryType, swingEndPosition);

      maximumAdjustmentDistance.set(maxSwingHeight - swingHeight);

      SwingGenerator swingGenerator;
      originalWaypoints.clear();
      adjustedWaypoints.clear();

      if (trajectoryType == TrajectoryType.OBSTACLE_CLEARANCE)
      {
         swingGenerator = twoWaypointSwingGenerator;
         numberOfWaypoints.set(2);
      }
      else
      {
         swingGenerator = oneWaypointSwingGenerator;
         numberOfWaypoints.set(1);
      }

         this.swingStartPosition.setMatchingFrame(swingStartPosition);
      swingGenerator.setInitialConditions(this.swingStartPosition, initialVelocity);

      this.swingEndPosition.setMatchingFrame(swingEndPosition);




      swingGenerator.setFinalConditions(this.swingEndPosition, touchdownVelocity);
      swingGenerator.setStepTime(stepDurationToCheckWith.getDoubleValue());

      double maxStepZ = Math.max(this.swingStartPosition.getZ(), this.swingEndPosition.getZ());

      midGroundPoint.setToZero();
      switch (trajectoryType)
      {
      case OBSTACLE_CLEARANCE:
         for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
         {
            FramePoint3D waypoint = originalWaypoints.add();
            waypoint.interpolate(swingStartPosition, swingEndPosition, twoWaypoingSwingProportions[i]);
            midGroundPoint.add(waypoint);
            waypoint.setZ(maxStepZ + swingHeight);
            adjustedWaypoints.add().set(waypoint);
         }
         break;
      default:
         for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
         {
            FramePoint3D waypoint = originalWaypoints.add();
            waypoint.interpolate(swingStartPosition, swingEndPosition, oneWaypointSwingProportion);
            midGroundPoint.add(waypoint);
            waypoint.addZ(swingHeight);
            adjustedWaypoints.add().set(waypoint);
         }
         break;
      }


      midGroundPoint.scale(0.5);

      adjustSwingEndIfCoincidentWithSwingStart();

      swingTrajectoryPlane.set(this.swingStartPosition, adjustedWaypoints.get(0), this.swingEndPosition);

      axisAngle.set(swingTrajectoryPlane.getNormal(), Math.PI / 2.0);
      rigidBodyTransform.setRotation(axisAngle);


      tempPlaneNormal.sub(this.swingStartPosition, this.swingEndPosition);
      rigidBodyTransform.transform(tempPlaneNormal);
      tempPlaneNormal.normalize();
      swingFloorPlane.set(this.swingStartPosition, tempPlaneNormal);

      tempPlaneNormal.sub(this.swingEndPosition, this.swingStartPosition);
      tempPointOnPlane.scaleAdd(collisionSphereRadius / tempPlaneNormal.length(), tempPlaneNormal, this.swingStartPosition);
      frontOfCollisionPlaneFacingEnd.set(tempPointOnPlane, tempPlaneNormal);

      tempPlaneNormal.sub(this.swingStartPosition, this.swingEndPosition);
      tempPointOnPlane.scaleAdd(collisionSphereRadius / tempPlaneNormal.length(), tempPlaneNormal, this.swingEndPosition);
      backOfCollisionPlaneFacingStart.set(tempPointOnPlane, tempPlaneNormal);

      status.set(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION);
      numberOfTriesCounter.resetCount();
      while (status.getEnumValue().equals(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType
               .values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                  .setIncludingFrame(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         }
         mostSevereCollisionType.set(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

         status.set(tryATrajectory(swingGenerator, planarRegionsList));
         updateVisualizer();
         numberOfTriesCounter.countOne();
      }

      return swingGenerator.computeAndGetMaxSpeed();
   }

   private TrajectoryType getTrajectoryType(TrajectoryType specifiedType, FramePoint3DReadOnly finalPosition)
   {
      if (specifiedType == null)
      {
         if (checkStepUpOrDown(finalPosition))
            return TrajectoryType.OBSTACLE_CLEARANCE;
         else
            return TrajectoryType.DEFAULT;
      }
      else
      {
         return specifiedType;
      }
   }


   private boolean checkStepUpOrDown(FramePoint3DReadOnly finalPosition)
   {
      double zDifference = Math.abs(finalPosition.getZ() - swingStartPosition.getZ());
      return zDifference > stepHeightChangeForObstacleAvoidance;
   }

   // TODO figure out a better solution for coincident points and replace this
   private void adjustSwingEndIfCoincidentWithSwingStart()
   {
      if (swingStartPosition.distance(swingEndPosition) < 1e-8)
         swingEndPosition.add(1e-4, 1e-4, 1e-4);
   }

   private SwingOverPlanarRegionsTrajectoryExpansionStatus tryATrajectory(SwingGenerator swingGenerator, PlanarRegionsList planarRegionsList)
   {
      swingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      swingGenerator.initialize();

      double phaseIncrement = 1.0 / numberOfCheckpoints.getIntegerValue();
      for (double phaseThroughTrajectory = 0.0; phaseThroughTrajectory < 1.0; phaseThroughTrajectory += phaseIncrement)
      {
         swingGenerator.compute(phaseThroughTrajectory);
         FramePoint3D frameTupleUnsafe = new FramePoint3D(trajectoryPosition);
         swingGenerator.getPosition(frameTupleUnsafe);
         trajectoryPosition.setMatchingFrame(frameTupleUnsafe);

         footCollisionSphere.setToZero(worldFrame);
         footCollisionSphere.setRadius(collisionSphereRadius);
         footCollisionSphere.getSphere3d().setPosition(trajectoryPosition);

         for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);
            planarRegion.getTransformToWorld(planarRegionTransform);
            planarRegionReferenceFrame.setTransformAndUpdate(planarRegionTransform);
            for (int j = 0; j < planarRegion.getNumberOfConvexPolygons(); j++)
            {
               framePlanarRegion.setIncludingFrame(planarRegionReferenceFrame, planarRegion.getConvexPolygon(j));

               boolean intersectionExists = sphereWithConvexPolygonIntersector.checkIfIntersectionExists(footCollisionSphere, framePlanarRegion);
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

               if (intersectionExists)
               {
                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_BELOW_IGNORE_PLANE);

                  if (swingFloorPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon())
                        && swingFloorPlane.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) > ignoreDistanceFromFloor)
                  {
                     updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_OUTSIDE_TRAJECTORY);

                     if ((frontOfCollisionPlaneFacingEnd.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon())
                           && backOfCollisionPlaneFacingStart.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()))
                           || midGroundPoint.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < midGroundPoint
                           .distance(trajectoryPosition))
                     {
                        updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.CRITICAL_INTERSECTION);

                        axisAngle.set(swingTrajectoryPlane.getNormal(), Math.PI * phaseThroughTrajectory);
                        rigidBodyTransform.setRotation(axisAngle);

//                        waypointAdjustmentVector.sub(swingStartPosition, swingEndPosition);
//                        waypointAdjustmentVector.normalize();
//                        rigidBodyTransform.transform(waypointAdjustmentVector);
//                        waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());

                        switch (numberOfWaypoints.getIntegerValue())
                        {
                        case 1:
                           waypointAdjustmentVector.sub(swingStartPosition, swingEndPosition);
                           waypointAdjustmentVector.normalize();
                           rigidBodyTransform.transform(waypointAdjustmentVector);
                           waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
//                           waypointAdjustmentVector.scale(1.0 - phaseIncrement);
                           adjustedWaypoints.get(0).add(waypointAdjustmentVector);
//                           adjustedWaypoints.get(0).add(waypointAdjustmentVector);
                           break;
                        case 2:

                           waypointAdjustmentVector.sub(swingStartPosition, swingEndPosition);
                           waypointAdjustmentVector.normalize();
                           rigidBodyTransform.transform(waypointAdjustmentVector);
                           waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
                           waypointAdjustmentVector.scale(1.0 - phaseIncrement);
                           adjustedWaypoints.get(0).add(waypointAdjustmentVector);

                           waypointAdjustmentVector.sub(swingStartPosition, swingEndPosition);
                           waypointAdjustmentVector.normalize();
                           rigidBodyTransform.transform(waypointAdjustmentVector);
                           waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
                           waypointAdjustmentVector.scale(phaseIncrement);
                           adjustedWaypoints.get(1).add(waypointAdjustmentVector);
                           break;
                        default:
                           throw new RuntimeException("not handled");
                        }

                        if (haveAnyAdjustmentsHitMaxDistance())
                        {
                           return SwingOverPlanarRegionsTrajectoryExpansionStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
                        }

                        return SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION;
                     }
                  }
               }
            }
         }

         updateVisualizer();
      }

      return SwingOverPlanarRegionsTrajectoryExpansionStatus.SOLUTION_FOUND;
   }


   private boolean haveAnyAdjustmentsHitMaxDistance()
   {
      for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
      {
         if (adjustedWaypoints.get(i).distance(originalWaypoints.get(i)) > maximumAdjustmentDistance.getDoubleValue())
            return true;
      }

      return false;
   }

   public void setCollisionSphereRadius(double collisionSphereRadius)
   {
      this.collisionSphereRadius = Math.max(0.0, collisionSphereRadius);
   }

   private void updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType collisionType)
   {
      if (collisionType.ordinal() > this.mostSevereCollisionType.getEnumValue().ordinal())
      {
         this.mostSevereCollisionType.set(collisionType);
      }
      if (footCollisionSphere.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < footCollisionSphere
            .distance(closestPolygonPointMap.get(collisionType)))
      {
         closestPolygonPointMap.get(collisionType).set(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon());
      }
   }

   public List<FramePoint3D> getExpandedWaypoints()
   {
      return adjustedWaypoints;
   }

   public SwingOverPlanarRegionsTrajectoryExpansionStatus getStatus()
   {
      return status.getEnumValue();
   }

   // VISULIZER METHODS

   public void updateVisualizer()
   {
      if (visualizer.isPresent())
      {
         visualizer.get().update(0.0);
      }
   }

   public FramePoint3DReadOnly getTrajectoryPosition()
   {
      return trajectoryPosition;
   }

   public void attachVisualizer(Updatable visualizer)
   {
      this.visualizer = Optional.of(visualizer);
   }

   public FramePoint3D getClosestPolygonPoint(SwingOverPlanarRegionsTrajectoryCollisionType collisionType)
   {
      return closestPolygonPointMap.get(collisionType);
   }

   public SwingOverPlanarRegionsTrajectoryCollisionType getMostSevereCollisionType()
   {
      return mostSevereCollisionType.getEnumValue();
   }

   public double getSphereRadius()
   {
      return footCollisionSphere.getRadius();
   }
}
