package us.ihmc.quadrupedBasics.heightMap;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class HeightMapEstimator
{
   private static final int defaultMaxBufferSize = 50;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoInteger maxBufferSize = new YoInteger("MaxPointBufferSize", registry);
   private final YoInteger currentPointBufferSize = new YoInteger("CurrentPointBufferSize", registry);

   private final List<PlanarRegion> planarRegionsBuffer = new ArrayList<>();
   private final List<HeightMapListener> heightMapListeners = new ArrayList<>();

   private final PlanarRegionBuilder regionBuilder = new PlanarRegionBuilder();

   public HeightMapEstimator(YoVariableRegistry parentRegistry)
   {
      maxBufferSize.set(defaultMaxBufferSize);

      parentRegistry.addChild(registry);
   }

   public void processIncomingPoint(Point3DReadOnly pointInWorld)
   {
      List<PlanarRegion> regionsToAdd = new ArrayList<>();

      PlanarRegion containingRegion = getContainingRegion(pointInWorld);
      if (containingRegion == null)
      {
         PlanarRegion nearestRegion = getNearestRegion(pointInWorld);
         regionsToAdd.add(createFromNearbyRegion(pointInWorld, nearestRegion));
      }
      else
      {
         regionsToAdd.addAll(createByDividingRegion(pointInWorld, containingRegion));
      }

      addRegionsToBuffer(regionsToAdd);
      reportUpdatedRegions();
   }

   public void setMaxBufferSize(int maxBufferSize)
   {
      this.maxBufferSize.set(maxBufferSize);
   }

   public List<PlanarRegion> getHeightMapRegionBuffer()
   {
      return planarRegionsBuffer;
   }

   public void attachHeightMapListener(HeightMapListener heightMapListener)
   {
      heightMapListeners.add(heightMapListener);
   }

   private PlanarRegion getContainingRegion(Point3DReadOnly pointInWorld)
   {
      for (PlanarRegion planarRegion : planarRegionsBuffer)
      {
         if (planarRegion.isPointInWorld2DInside(pointInWorld))
            return planarRegion;
      }

      return null;
   }

   private PlanarRegion getNearestRegion(Point3DReadOnly pointInWorld)
   {
      double shortestDistance = Double.POSITIVE_INFINITY;
      PlanarRegion closestRegion = null;
      for (PlanarRegion planarRegion : planarRegionsBuffer)
      {
         double distanceToRegion = planarRegion.distanceToPointInWorldByProjectionOntoXYPlane(pointInWorld);
         if (distanceToRegion < shortestDistance)
         {
            shortestDistance = distanceToRegion;
            closestRegion = planarRegion;
         }
      }

      return closestRegion;
   }

   private PlanarRegion createFromNearbyRegion(Point3DReadOnly pointInWorld, PlanarRegion nearestRegion)
   {
      Point3D closestPoint = new Point3D();
      Point3D secondClosestPoint = new Point3D();
      double closestDistance = Double.POSITIVE_INFINITY;
      double secondClosestDistance = Double.POSITIVE_INFINITY;

      for (ConvexPolygon2D convexPolygon : nearestRegion.getConvexPolygons())
      {
         for (Point2DReadOnly vertexInLocal : convexPolygon.getVertexBufferView())
         {
            Point3D vertexInWorld = new Point3D(vertexInLocal);
            nearestRegion.transformFromLocalToWorld(vertexInWorld);

            double distance = vertexInWorld.distance(pointInWorld);
            if (distance < closestDistance)
            {
               secondClosestPoint = closestPoint;
               secondClosestDistance = closestDistance;

               closestPoint = vertexInWorld;
               closestDistance = distance;
            }
            else if (distance < secondClosestDistance)
            {
               secondClosestPoint = vertexInWorld;
               secondClosestDistance = distance;
            }
         }
      }

      regionBuilder.reset();
      regionBuilder.addNodes(pointInWorld, closestPoint, secondClosestPoint);

      // TODO set the id
//      regionBuilder.setId(id);
      return regionBuilder.buildPlanarRegion();
   }

   private List<PlanarRegion> createByDividingRegion(Point3DReadOnly pointInWorld, PlanarRegion regionToDivide)
   {
      Point2D pointInLocal = new Point2D(pointInWorld);
      regionToDivide.transformFromWorldToLocal(pointInLocal);

      ConvexPolygon2D polygonToDivide = null;
      List<ConvexPolygon2D> polygonsToKeep = new ArrayList<>();

      for (ConvexPolygon2D polygon : regionToDivide.getConvexPolygons())
      {
         if (polygon.isPointInside(pointInLocal))
            polygonToDivide = polygon;
         else
            polygonsToKeep.add(polygon);
      }

      if (polygonToDivide == null)
         throw new RuntimeException("Should have found a point inside to get here.");

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      regionToDivide.getTransformToWorld(transformToWorld);
      PlanarRegion planarRegionToKeep = new PlanarRegion(transformToWorld, polygonsToKeep);

      List<PlanarRegion> planarRegions = new ArrayList<>();
      planarRegions.add(planarRegionToKeep);

      for (int i = 0; i < polygonToDivide.getNumberOfVertices(); i++)
      {
         Point3D vertex = new Point3D(polygonToDivide.getVertex(i));
         Point3D nextVertex = new Point3D(polygonToDivide.getNextVertex(i));

         transformToWorld.transform(vertex);
         transformToWorld.transform(nextVertex);
         regionBuilder.reset();
         regionBuilder.addNodes(pointInWorld, vertex, nextVertex);
         // TODO set the id
//         regionBuilder.setId(id);

         planarRegions.add(regionBuilder.buildPlanarRegion());
      }

      return planarRegions;
   }

   // FIXME clean this guy up to have it actually use the max buffer size
   private void addRegionsToBuffer(List<PlanarRegion> regionsToAdd)
   {
      planarRegionsBuffer.addAll(regionsToAdd);
   }

   private void reportUpdatedRegions()
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegionsBuffer);
      for (HeightMapListener heightMapListener : heightMapListeners)
         heightMapListener.reportNewHeightMap(planarRegionsList);
   }
}
