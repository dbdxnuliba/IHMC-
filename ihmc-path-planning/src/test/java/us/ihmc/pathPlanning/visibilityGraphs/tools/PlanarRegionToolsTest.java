package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.*;
import static org.junit.Assert.assertNull;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTest;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testTruncatePlanarRegionIfIntersectingWithPlane() throws Exception
   {
      Point3D groundOrigin = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D squareOrigin = new Point3D(0.0, 0.0, -0.001);
      Vector3D squareNormal = new Vector3D(0.0, -1.0, 0.0);
      AxisAngle squareOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(squareNormal);
      RigidBodyTransform squarePose = new RigidBodyTransform(squareOrientation, squareOrigin);

      double squareSide = 4.0;

      Point2D[] concaveHullVertices = {new Point2D(0.0, 0.0), new Point2D(0.0, squareSide), new Point2D(squareSide, squareSide), new Point2D(squareSide, 0.0)};
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      convexPolygons.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices)));
      PlanarRegion verticalSquare = new PlanarRegion(squarePose, concaveHullVertices, convexPolygons);

      Point3D[] expectedVerticesInWorld = Arrays.stream(concaveHullVertices).map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);
      expectedVerticesInWorld[0].addZ(0.001);
      expectedVerticesInWorld[3].addZ(0.001);

      PlanarRegion truncatedSquare = PlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, verticalSquare, 0.05, null);
      RigidBodyTransform truncatedTransform = new RigidBodyTransform();
      truncatedSquare.getTransformToWorld(truncatedTransform);
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(squarePose, truncatedTransform, EPSILON);

      Point3D[] actualVerticesInWorld = Arrays.stream(truncatedSquare.getConcaveHull()).map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);

      assertEquals(expectedVerticesInWorld.length, actualVerticesInWorld.length);

      for (int i = 0; i < expectedVerticesInWorld.length; i++)
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedVerticesInWorld[i], actualVerticesInWorld[i], EPSILON);
   }

   public static Point3D toWorld(Point2D point2D, Transform transformToWorld)
   {
      Point3D inWorld = new Point3D(point2D);
      transformToWorld.transform(inWorld);
      return inWorld;
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   public void testIsInsidePolygon() throws Exception
   {
      Random random = new Random(324534L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with convex polygon
         List<? extends Point2DReadOnly> convexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 10.0, 100);
         int hullSize = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(convexPolygon2D);
         boolean clockwiseOrdered = random.nextBoolean();
         if (!clockwiseOrdered)
            Collections.reverse(convexPolygon2D.subList(0, hullSize));

         Point2DReadOnly[] convexPolygon2DArray = convexPolygon2D.subList(0, hullSize).toArray(new Point2DReadOnly[convexPolygon2D.size()]);

         Point2D centroid = new Point2D();
         EuclidGeometryPolygonTools.computeConvexPolyong2DArea(convexPolygon2D, hullSize, clockwiseOrdered, centroid);
         int vertexIndex = random.nextInt(hullSize);
         int nextVertexIndex = EuclidGeometryPolygonTools.next(vertexIndex, hullSize);
         Point2DReadOnly vertex = convexPolygon2D.get(vertexIndex);
         Point2DReadOnly nextVertex = convexPolygon2D.get(nextVertexIndex);

         Point2D pointOnEdge = new Point2D();
         pointOnEdge.interpolate(vertex, nextVertex, random.nextDouble());

         double alphaOutside = nextDouble(random, 1.0, 3.0);
         Point2D outsidePoint = new Point2D();
         outsidePoint.interpolate(centroid, pointOnEdge, alphaOutside);
         assertFalse(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, outsidePoint));

         double alphaInside = nextDouble(random, 0.0, 1.0);
         Point2D insidePoint = new Point2D();
         insidePoint.interpolate(centroid, pointOnEdge, alphaInside);
         assertTrue(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, insidePoint));
      }
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testIsInsidePolygonBug1() throws Exception
   {
      Point2D[] polygon = {new Point2D(-0.3, 0.5), new Point2D(0.3, 0.5), new Point2D(0.3, -0.5), new Point2D(-0.3, -0.5)};
      Point2D pointToCheck = new Point2D(-2.0, 0.5);

      assertFalse(PlanarRegionTools.isPointInsidePolygon(polygon, pointToCheck));
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testProjectPointToPlanesVertically()
   {
      Random random = new Random(1738L);

      // first test stacked regions
      List<PlanarRegion> listOfPlanarRegions = new ArrayList<>();
      int numberOfRegions = 5;
      ConvexPolygon2D polygonInWorld = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 10.0, 10);
      polygonInWorld.update();
      Point2D[] concaveHull = polygonInWorld.getPolygonVerticesView().toArray(new Point2D[0]);
      Point2DReadOnly centroidInWorld = polygonInWorld.getCentroid();

      List<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygonInWorld);


      double layerSeparation = 0.10;
      for (int i = 0; i < numberOfRegions; i++)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(0.0, 0.0, i * layerSeparation);
         PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
         listOfPlanarRegions.add(planarRegion);
      }

      // test with point at centroid
      Point3DReadOnly pointToProject = new Point3D(centroidInWorld);

      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      Point3D expectedPoint = new Point3D(pointToProject);
      expectedPoint.setZ(layerSeparation * (numberOfRegions - 1));

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, projectedPoint, 1e-6);

      // test with point outside bounds
      pointToProject = new Point3D(15.0, 15.0, 100.0);

      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      assertNull(projectedPoint);

      // test two slightly overlapping regions
      ConvexPolygon2D polygonA = new ConvexPolygon2D();
      polygonA.addVertex(0.5, 0.5);
      polygonA.addVertex(0.5, -0.5);
      polygonA.addVertex(-0.5, 0.5);
      polygonA.addVertex(-0.5, -0.5);
      polygonA.update();
      ConvexPolygon2D polygonB = new ConvexPolygon2D();
      polygonB.addVertex(0.5, 0.5);
      polygonB.addVertex(0.5, -0.5);
      polygonB.addVertex(-0.5, 0.5);
      polygonB.addVertex(-0.5, -0.5);
      polygonB.update();
      polygons.clear();
      polygons.add(polygonA);
      polygons.add(polygonB);
      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();
      transformA.setTranslation(new Vector3D(0.4, 0.0, 0.0));
      transformB.setTranslation(new Vector3D(-0.4, 0.0, 0.1));

      PlanarRegion regionA = new PlanarRegion(transformA, polygonA);
      PlanarRegion regionB = new PlanarRegion(transformB, polygonB);
      listOfPlanarRegions.clear();
      listOfPlanarRegions.add(regionA);
      listOfPlanarRegions.add(regionB);

      // middle
      pointToProject = new Point3D();
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.0, 0.0, 0.1), projectedPoint, 1e-6);

      // on the edge
      pointToProject = new Point3D(0.1 - 1e-5, 0.0, 0.0);
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.1 - 1e-5, 0.0, 0.1), projectedPoint, 1e-6);

      // past edge
      pointToProject = new Point3D(0.2, 0.0, 0.0);
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.2, 0.0, 0.0), projectedPoint, 1e-6);

      pointToProject = new Point3D(-0.2, 0.0, 0.0);
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-0.2, 0.0, 0.1), projectedPoint, 1e-6);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testFilterPlanarRegionsWithBoundingCircle()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 100; iter++)
      {
         double maxRegionDimension = 5.0;
         double maxRegionDistanceForGuaranteedOutOfBounds = Math.sqrt(2.0 * maxRegionDimension * maxRegionDimension);

         int numberOfPoints = 5;
         ConvexPolygon2D planarRegionPolygonA = new ConvexPolygon2D();
         ConvexPolygon2D planarRegionPolygonB = new ConvexPolygon2D();
         Point2D[] concaveHull = new Point2D[2 * numberOfPoints];
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            planarRegionPolygonA.addVertex(pointA);
            planarRegionPolygonB.addVertex(pointB);

            concaveHull[2 * i] = pointA;
            concaveHull[2 * i + 1] = pointB;
         }
         planarRegionPolygonA.update();
         planarRegionPolygonB.update();

         double maxRegionDistance = Math.max(findFurthestPointFromOrigin(planarRegionPolygonA), findFurthestPointFromOrigin(planarRegionPolygonB));

         List<ConvexPolygon2D> polygons = new ArrayList<>();
         polygons.add(planarRegionPolygonA);
         polygons.add(planarRegionPolygonB);

         int numberOfRegionsWithinDistance = random.nextInt(10);
         int numberOfRegionsOutsideOfDistance = random.nextInt(20);

         Point3D randomOrigin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         List<PlanarRegion> regionsWithinDistanceExpected = new ArrayList<>();
         List<PlanarRegion> regionsOutsideDistance = new ArrayList<>();
         List<PlanarRegion> allRegions = new ArrayList<>();
         for (int i = 0; i < numberOfRegionsWithinDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, -0.2 * maxRegionDistance, 0.2 * maxRegionDistance);
            translation.add(randomOrigin);
            transform.setTranslation(translation);
            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsWithinDistanceExpected.add(planarRegion);
            allRegions.add(planarRegion);
         }

         for (int i = 0; i < numberOfRegionsOutsideOfDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            double xSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double ySign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double zSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;

            double xTranslation = xSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double yTranslation = ySign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double zTranslation = zSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            Vector3D translation = new Vector3D(xTranslation, yTranslation, zTranslation);
            translation.add(randomOrigin);

            transform.setTranslation(translation);

            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsOutsideDistance.add(planarRegion);
            allRegions.add(planarRegion);
         }

         List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(randomOrigin), maxRegionDistance, allRegions);

         assertEquals(regionsWithinDistanceExpected.size(), regionsWithinDistance.size());
         for (int i = 0; i < regionsWithinDistance.size(); i++)
         {
            assertTrue(regionsWithinDistanceExpected.contains(regionsWithinDistance.get(i)));
            assertFalse(regionsOutsideDistance.contains(regionsWithinDistance.get(i)));
         }
      }
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testFilterPlanarRegionsWithBoundingCirclePointWithinBigRegion()
   {
      ConvexPolygon2D polygon2D = new ConvexPolygon2D();
      polygon2D.addVertex(10.0, 10.0);
      polygon2D.addVertex(10.0, -10.0);
      polygon2D.addVertex(-10.0, -10.0);
      polygon2D.addVertex(-10.0, 10.0);
      polygon2D.update();
      List<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon2D);

      Point2D[] concaveHull = polygon2D.getPolygonVerticesView().toArray(new Point2D[0]);

      RigidBodyTransform transform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
      List<PlanarRegion> planarRegionList = new ArrayList<>();
      planarRegionList.add(planarRegion);


      // at middle of planar region
      List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));

      // outside the planar region, but still within the distance
      regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(10.5, 0.0), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrivialCase() throws Exception
   {
      // polygons forming a "|"-shaped region.
      List<ConvexPolygon2D> region1ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(5.0, 1.0);
      polygon1.addVertex(5.0, -1.0);
      polygon1.addVertex(-5.0, -1.0);
      polygon1.addVertex(-5.0, 1.0);

      region1ConvexPolygons.add(polygon1);
      for (ConvexPolygon2D convexPolygon : region1ConvexPolygons)
         convexPolygon.update();


      // polygons forming a "--"-shaped region.
      List<ConvexPolygon2D> region2ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(1.0, 5.0);
      polygon2.addVertex(1.0, -5.0);
      polygon2.addVertex(-1.0, -5.0);
      polygon2.addVertex(-1.0, 5.0);

      region2ConvexPolygons.add(polygon2);
      for (ConvexPolygon2D convexPolygon : region2ConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform region1Transform = new RigidBodyTransform();
      RigidBodyTransform region2Transform = new RigidBodyTransform();

      region2Transform.setTranslation(0.0, 0.0, 1.0);

      PlanarRegion planarRegion1 = new PlanarRegion(region1Transform, region1ConvexPolygons);
      PlanarRegion planarRegion2 = new PlanarRegion(region2Transform, region2ConvexPolygons);
      List<PlanarRegion> planarRegions = new ArrayList<>();
      planarRegions.add(planarRegion1);
      planarRegions.add(planarRegion2);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);

      List<PlanarRegion> result;

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

      // Do a bunch of trivial queries with findPlanarRegionsIntersectingPolygon(ConvexPolygon2d convexPolygon)
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(convexPolygon, planarRegionsList);
      assertEquals(2, result.size());
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(2.0, 0.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(-2.0, 0.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(0.0, 2.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(0.0, -2.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(2.0, 2.0, convexPolygon), planarRegionsList);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(2.0, -2.0, convexPolygon), planarRegionsList);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(-2.0, -2.0, convexPolygon), planarRegionsList);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(-2.0, 2.0, convexPolygon), planarRegionsList);
      assertNull(result);
   }

   static ConvexPolygon2DBasics translateConvexPolygon(double xTranslation, double yTranslation, ConvexPolygon2DReadOnly convexPolygon)
   {
      Vector2D translation = new Vector2D(xTranslation, yTranslation);
      return convexPolygon.translateCopy(translation);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testFilterPlanarRegionsWithBoundingCapsule()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 100; iter++)
      {
         double maxRegionDimension = 5.0;
         double maxRegionDistanceForGuaranteedOutOfBounds = Math.sqrt(2.0 * maxRegionDimension * maxRegionDimension);

         int numberOfPoints = 5;
         ConvexPolygon2D planarRegionPolygonA = new ConvexPolygon2D();
         ConvexPolygon2D planarRegionPolygonB = new ConvexPolygon2D();
         Point2D[] concaveHull = new Point2D[2 * numberOfPoints];
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            planarRegionPolygonA.addVertex(pointA);
            planarRegionPolygonB.addVertex(pointB);

            concaveHull[2 * i] = pointA;
            concaveHull[2 * i + 1] = pointB;
         }
         planarRegionPolygonA.update();
         planarRegionPolygonB.update();

         double maxRegionDistance = Math.max(findFurthestPointFromOrigin(planarRegionPolygonA), findFurthestPointFromOrigin(planarRegionPolygonB));

         List<ConvexPolygon2D> polygons = new ArrayList<>();
         polygons.add(planarRegionPolygonA);
         polygons.add(planarRegionPolygonB);

         int numberOfRegionsWithinDistance = random.nextInt(10);
         int numberOfRegionsOutsideOfDistance = random.nextInt(20);

         Point3D randomOriginStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D randomOriginEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         LineSegment3D randomSegment = new LineSegment3D(randomOriginStart, randomOriginEnd);
         Point3D midpoint = new Point3D();
         midpoint.interpolate(randomOriginStart, randomOriginEnd, 0.5);

         List<PlanarRegion> regionsWithinDistanceExpected = new ArrayList<>();
         List<PlanarRegion> regionsOutsideDistance = new ArrayList<>();
         List<PlanarRegion> allRegions = new ArrayList<>();
         for (int i = 0; i < numberOfRegionsWithinDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, -0.2 * maxRegionDistance, 0.2 * maxRegionDistance);
            translation.add(midpoint);
            transform.setTranslation(translation);
            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsWithinDistanceExpected.add(planarRegion);
            allRegions.add(planarRegion);
         }

         for (int i = 0; i < numberOfRegionsOutsideOfDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            double xSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double ySign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double zSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;

            double xTranslation = xSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double yTranslation = ySign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double zTranslation = zSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            Vector3D translation = new Vector3D(xTranslation, yTranslation, zTranslation);
            double distanceStart = randomOriginStart.distance(new Point3D(translation));
            double distanceEnd = randomOriginStart.distance(new Point3D(translation));

            if (distanceStart < distanceEnd)
               translation.add(randomOriginStart);
            else
               translation.add(randomOriginEnd);

            transform.setTranslation(translation);

            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsOutsideDistance.add(planarRegion);
            allRegions.add(planarRegion);
         }

         List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(randomSegment, maxRegionDistance, allRegions);

         assertEquals(regionsWithinDistanceExpected.size(), regionsWithinDistance.size());
         for (int i = 0; i < regionsWithinDistance.size(); i++)
         {
            assertTrue(regionsWithinDistanceExpected.contains(regionsWithinDistance.get(i)));
            assertFalse(regionsOutsideDistance.contains(regionsWithinDistance.get(i)));
         }
      }
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testFilterPlanarRegionsWithBoundingCapsulePointWithinBigRegion()
   {
      ConvexPolygon2D polygon2D = new ConvexPolygon2D();
      polygon2D.addVertex(10.0, 10.0);
      polygon2D.addVertex(10.0, -10.0);
      polygon2D.addVertex(-10.0, -10.0);
      polygon2D.addVertex(-10.0, 10.0);
      polygon2D.update();
      List<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon2D);

      Point2D[] concaveHull = polygon2D.getPolygonVerticesView().toArray(new Point2D[0]);

      RigidBodyTransform transform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
      List<PlanarRegion> planarRegionList = new ArrayList<>();
      planarRegionList.add(planarRegion);


      // at middle of planar region
      List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(new Point3D(0.1, 0.0, 0.0), new Point3D(-0.1, 0.0, 0.0), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));

      // outside the planar region, but still within the distance
      regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(new Point3D(10.5, 0.1, 0.0), new Point3D(10.5, -0.1, 0.0), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));
   }

   private static double findFurthestPointFromOrigin(ConvexPolygon2D polygon)
   {
      Point2D origin = new Point2D();
      double distance = 0.0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         distance = Math.max(distance, polygon.getVertex(i).distance(origin));
      }

      return distance;
   }
}