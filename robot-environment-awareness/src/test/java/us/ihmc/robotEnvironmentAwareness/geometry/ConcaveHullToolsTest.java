package us.ihmc.robotEnvironmentAwareness.geometry;

import org.junit.jupiter.api.Test;
import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import com.vividsolutions.jts.geom.MultiPoint;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;

public class ConcaveHullToolsTest extends ConcaveHullTestBasics
{
	private static final boolean DEBUG = true;
	private static final double EPS = 1.0e-6;

	public ConcaveHullToolsTest()
	{
		VISUALIZE = false;
	}

//	@Test
//	public void testPointCloudWithSurroundingLineConstraints() throws Exception
//	{
//		initializeBaseClass();
//
//		MultiPoint multiPoint = SimpleConcaveHullFactory.filterAndCreateMultiPoint(getPointcloud2D(), getLineConstraints2D(), .001);
//
//		PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud3D);
//		data.addIntersections(lineConstraints3D);
//		if(DEBUG) System.out.println("ConcaveHullToolsTest: data: " + data.toString());
//
//		ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
//		parameters.setTriangulationTolerance(1.0e-5);
//		parameters.setEdgeLengthThreshold(0.15);
//		if(DEBUG) System.out.println("ConcaveHullToolsTest: parameters: " + parameters.toString());
//
//		messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);
//
//		AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
//		// System.out.println("ConcaveHullToolsTest: output: " + output.get().toString());
//
//		messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));
//
//		while (output.get() == null)
//			ThreadTools.sleep(100);
//
//		ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
//		if(DEBUG) System.out.println("ConcaveHullToolsTest: concaveHullCollection: " + concaveHullCollection.getConcaveHulls());
//
//		int numberOfHulls = concaveHullCollection.getNumberOfConcaveHulls();
//
//		if(DEBUG) System.out.printf("ConcaveHullToolsTest: numberOfHulls = %d ", numberOfHulls);
//
//		assertEquals(numberOfHulls, 2);
//
//		int hullCount = 0;
//		ConcaveHull concaveHull = null;
//		ConcaveHull convexHull = null;
//		ConcaveHullPocket pocket = null;
//
//		for (ConcaveHull hull : concaveHullCollection)
//		{
//			int num = hull.getNumberOfVertices();
//			if(DEBUG) System.out.printf("\nConcaveHullToolsTest: Hull %d is %s, number of vertices = %d", hullCount, hull.isHullConvex() ? "Convex" : "Concave", num);
//
//			if (hullCount == 0)
//			{
//				assert (hull.isHullConvex() == false);
//				assert (num == 24);
//				concaveHull = hull;
//			}
//			if (hullCount == 1)
//			{
//				assert (hull.isHullConvex() == true);
//				assert (num == 4);
//				convexHull = hull;
//			}
//			hullCount++;
//		}
//
//		pocket = convexHull.findFirstConcaveHullPocket();
//		assert (pocket == null);
//
//		pocket = concaveHull.findFirstConcaveHullPocket();
//		assert (pocket != null);
//		assertEquals(pocket.getMaxDepth(), 0.42956, EPS);
//
//		int vertex = 0;
//		do
//		{
//			pocket = concaveHull.computeConcaveHullPocket(vertex);
//			if (pocket != null)
//			{
//				assert (vertex == 2);
//				assertEquals(pocket.getMaxDepth(), 0.429561, EPS);
//				if (DEBUG) System.out.printf("testPointCloudWithSurroundingLineConstraints: pocket at vertex %d depth is %f", vertex, pocket.getMaxDepth());
//			}
//			vertex++;
//		}
//		while (pocket == null);
//		assertEquals(pocket.getMaxDepth(), 0.42956, EPS);
//	}

//	@Test
//	public void testEnsureCounterClockwiseOrdering()
//	{

//		initializeBaseClass();

//		if (sombreroInitialized)
//		{
//			List<Point2D> counterClockwiseVertices = new ArrayList<Point2D>();
//			counterClockwiseVertices.addAll(sombrero);
//			ConcaveHullTools.ensureCounterClockwiseOrdering(counterClockwiseVertices);
//			int n = sombrero.size();
//			for (int i = 0; i < n; i++)
//			{
//				Point2DReadOnly p = sombrero.get(i);
//				Point2DReadOnly q = counterClockwiseVertices.get(n-1-i);
//				assert(p.equals(q));
//            if (DEBUG)
//               System.out.printf("\ntestEnsureCounterClockwiseOrdering: false");
//			}
//		}
//	}

	@Test
	public void testComputePerimeter()
	{
		initializeBaseClass();

		List<? extends Point2DReadOnly> concaveHullVertices = sombrero;
		double perimeter = ConcaveHullTools.computePerimeter(concaveHullVertices);

		assertEquals(perimeter, 992.217367, EPS); // value for createMexicanHat(-5, 5, 51);
		if (DEBUG)
			System.out.printf("\ntestComputePerimeter: %f", perimeter);
	}

	@Test
	public void testRemoveSuccessiveDuplicateVertices() throws Exception
	{
		Random random = new Random();
		initializeBaseClass();

		// create a duplicate of the original sombrero and verify it...
		List<Point2D> newSombrero = new ArrayList<Point2D>();
		newSombrero.addAll(sombrero);
		assert (newSombrero.size() == sombrero.size());
		for (int i = 0; i < newSombrero.size(); i++)
			assert( newSombrero.get(i).equals(sombrero.get(i)));
		
		// check the null case, that there are no duplicates currently in the sombrero
		int successiveDuplicateVertices = ConcaveHullTools.removeSuccessiveDuplicateVertices(sombrero);
		assert (successiveDuplicateVertices == 0);

		// randomly insert a mess of successive duplicates
		int m = ITERATIONS/10;
		int n = sombrero.size();
		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				int pos = random.nextInt(n);
				newSombrero.add(pos, newSombrero.get(pos));
			}
		}
		
		// now remove them all and check that the number removed matches the number created
		successiveDuplicateVertices = ConcaveHullTools.removeSuccessiveDuplicateVertices(newSombrero);
		assert (successiveDuplicateVertices == m * n);
		
		// finally, check that the new sombrero still matches the original sombrero
		assert (newSombrero.size() == sombrero.size());
		for (int i = 0; i < newSombrero.size(); i++)
			assert( newSombrero.get(i).equals(sombrero.get(i)));
	
		if (DEBUG) System.out.printf("\ntestRemoveSuccessiveDuplicateVertices: randomly inserted and later removed %d duplicate pts correctly", successiveDuplicateVertices);

	}

	@Test
	public void testComputeConcaveHullPocket()
	{
		Random random = new Random();
		initializeBaseClass();

		List<? extends Point2DReadOnly> vertices = sombrero;
		ConcaveHullPocket pocketToPack = new ConcaveHullPocket();
		for (int i = 0; i < vertices.size(); i++)
		{
			boolean concaveHullPocket = ConcaveHullTools.computeConcaveHullPocket(i, pocketToPack, vertices);
			//if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: i = %d %d %d ", i, min1, min2);
			if (i == min1)
			{
				assert (concaveHullPocket == true);
				assert (pocketToPack.getDeepestVertexIndex() == 18);
				assertEquals(pocketToPack.getDeepestVertex().getX(), -1.600000, EPS);
				assertEquals(pocketToPack.getDeepestVertex().getY(), -0.376192, EPS);
				assert (pocketToPack.getEndBridgeIndex() == 26);
				assertEquals(pocketToPack.getEndBridgeVertex().getX(), 0.0, EPS);
				assertEquals(pocketToPack.getEndBridgeVertex().getY(), 0.867325, EPS);

				if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: found min1 at %d", pocketToPack.getDeepestVertexIndex());
				if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: deepestVertex =  %f %f", pocketToPack.getDeepestVertex().getX(), pocketToPack.getDeepestVertex().getY());
				if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: endBridgeVertex at %d", pocketToPack.getEndBridgeIndex());
				if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: endBridgeVertex =  %f %f", pocketToPack.getEndBridgeVertex().getX(), pocketToPack.getEndBridgeVertex().getY());
			}
			
			if (i == min2)
         {
            assert (concaveHullPocket == true);
            assert (pocketToPack.getDeepestVertexIndex() == 34);
            assertEquals(pocketToPack.getDeepestVertex().getX(), 1.600000, EPS);
            assertEquals(pocketToPack.getDeepestVertex().getY(), -0.376192, EPS);
            assert (pocketToPack.getEndBridgeIndex() == 51);
            assertEquals(pocketToPack.getEndBridgeVertex().getX(), 5.000000, EPS);
            assertEquals(pocketToPack.getEndBridgeVertex().getY(), -0.000078, EPS);

          if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: found min2 at %d", pocketToPack.getDeepestVertexIndex());
          if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: deepestVertex =  %f %f", pocketToPack.getDeepestVertex().getX(), pocketToPack.getDeepestVertex().getY());
          if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: endBridgeVertex at %d", pocketToPack.getEndBridgeIndex());
          if(DEBUG) System.out.printf("\ntestComputeConcaveHullPocket: endBridgeVertex =  %f %f", pocketToPack.getEndBridgeVertex().getX(), pocketToPack.getEndBridgeVertex().getY());
         }
			
		}

		if (DEBUG) System.out.printf("\ntestComputeConcaveHullPocket");
	}

	@Test
	public void testFindConcaveHullPockets()
	{
		initializeBaseClass();

		double depthThreshold = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D;
		Set<ConcaveHullPocket> concaveHullPockets = ConcaveHullTools.findConcaveHullPockets(concaveHullVertices, depthThreshold);

		assert (concaveHullPockets != null);
		assert (concaveHullPockets.size() == 6);

		//public static Set<ConcaveHullPocket> findConcaveHullPockets(List<? extends Point2DReadOnly> concaveHullVertices, double depthThreshold)
		if (DEBUG)
			System.out.println("\ntestFindConcaveHullPockets: " + concaveHullPockets.size());
	}

	@Test
	public void testFindClosestIntersectionWithRay()
	{
		initializeBaseClass();

		Point2DReadOnly rayOrigin = new Point2D(0.0, max+1.0);
		Vector2DReadOnly rayDirection = new Vector2D(1.0, -10.0);
		int startSearchIndex = 0, endSearchIndex = sombrero.size()-1;
		Point2DBasics intersectionToPack = (Point2DBasics) new Point2D(0.0, 0.0);

		int closestIntersectionWithRay = ConcaveHullTools.findClosestIntersectionWithRay(rayOrigin, rayDirection, startSearchIndex, endSearchIndex,
		                                                                                 sombrero, intersectionToPack);
		//assert (closestIntersectionWithRay == 9);

		if (DEBUG)
			System.out.printf("\ntestFindClosestIntersectionWithRay: %d", closestIntersectionWithRay);
	}

	@Test
	public void testFirstConcaveHullPocketInefficient()
	{
		initializeBaseClass();

		List<? extends Point2DReadOnly> concaveHullVertices = sombrero;
		ConcaveHullPocket firstConcaveHullPocketInefficient = ConcaveHullTools.findFirstConcaveHullPocketInefficient(concaveHullVertices);
		assert (firstConcaveHullPocketInefficient != null);
		if (firstConcaveHullPocketInefficient != null)
		{
			assert (firstConcaveHullPocketInefficient.getDeepestVertexIndex() == 18);
			assertEquals(firstConcaveHullPocketInefficient.getMaxDepth(), 0.9517329, EPS);
			if (DEBUG)
				System.out.print("\ntestFirstConcaveHullPocketInefficient: DeepestVertexIndex, MaxDepth = "
				      + firstConcaveHullPocketInefficient.getDeepestVertexIndex() + ", " + firstConcaveHullPocketInefficient.getMaxDepth());
		}
		else 
			assert(false);
	}

	@Test
	public void testFindInnerClosestEdgeToVertex()
	{
		initializeBaseClass();

		int vertexIndex = 0, deadIndexRegion = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = sombrero;
		Point2DBasics closestPointToPack = new Point2D(0, 0);
		int innerClosestEdgeToVertex = ConcaveHullTools.findInnerClosestEdgeToVertex(vertexIndex, deadIndexRegion, concaveHullVertices, closestPointToPack);
		assert (innerClosestEdgeToVertex == 1);
		if (DEBUG)
			System.out.printf("\ntestFindInnerClosestEdgeToVertex: %d", innerClosestEdgeToVertex);
	}

	@Test
	public void testIsVertexPreventingKink()
	{
		boolean baseClassInitialized = initializeBaseClass();

		if (sombreroInitialized)
		{
			int[] bitMap = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,0,0,0,0};
			
         int n = sombrero.size();
         String str = String.format("int[%d] bitMap = {", n);

			for (int i = 0; i < n; i++)
			{
				// push the maximum to the right a little and observe the effect...
				// it should become a vertex preventing a kink.
				int target = max;
//				if (target >= 0 && target < n)
//				{
//					Point2D pt = sombrero.get(target);
//					pt.setX(pt.getX() + 0);
//					sombrero2D.set(target, pt);
//				}

				int bit = 0;
				boolean isVertexPreventingKink = ConcaveHullTools.isVertexPreventingKink(i, sombrero);

				if (isVertexPreventingKink) // 0 = unlabeled
					bit = 1; // 1 = isVertexPreventingKink
				if (i == min1 || i == min2) // 2 = a minimum
					bit = (bit == 1) ? 4 : 2; // 3 = a maximum
				if (i == max) // 4 = minimum and isVertexPreventingKink
					bit = (bit == 1) ? 5 : 3; // 5 = maximum and isVertexPreventingKink

				str += String.format("%d", bit);
				if (i < n - 1)
					str += String.format(",");
				else
					str += String.format("};");

				assertEquals(isVertexPreventingKink, bitMap[i]==1 || bitMap[i]==4 || bitMap[i]==5);
			}
			if (DEBUG)
				System.out.printf("\ntestIsVertexPreventingKink: " + str);
		}
		else
			assert (sombreroInitialized == true);
	}

	@Test
	public void testIsAlmostConvexAtVertex()
	{
		initializeBaseClass();

		int vertexIndex = 0;
		double angleTolerance = Math.toRadians(5);

		if (sombreroInitialized)
		{
			int[] bitMap = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,0,0,0,0,1,1,1,1,5,1,1,1,1,1,1,1,0,2,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,0,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,0,0,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,0,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0,1,0,1,1,1,1,0,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,0,1,1,0,1,0,0,1,1,1,0,1,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,1,1,0,1,0,1,1,1,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1,1,0,1,0,1,1,0,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,0,1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,0,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,1,1,1,1,1,1,1,0,1,0,0,0,1,1,1,1,0,0,1,0,0,1,1,1,1,1,1,1,0,1,1,1};
			List<? extends Point2DReadOnly> concaveHullVertices = sombrero;

			int n = concaveHullVertices.size();
			String str = String.format("int[%d] bitMap = {", n);

			for (int i = 0; i < n; i++)
			{

				boolean isAlmostConvexAtVertex = ConcaveHullTools.isAlmostConvexAtVertex(i, angleTolerance, concaveHullVertices);

				int bit = 0;
				if (isAlmostConvexAtVertex) // 0 = unlabeled
					bit = 1; // 1 = isAlmostConvexAtVertex

				if (i == min1 || i == min2) 	// 0b0x1x = a minimum
					bit |= 2; 						// 0b01xx = a maximum
				if (i == max) 						
					bit |= 4; 						
				bitMap[i] = bit;
				
				str += String.format("%d", bit);
				if (i < n - 1)
					str += String.format(",");
				else
					str += String.format("};");

			   assertEquals(isAlmostConvexAtVertex?1:0, bitMap[i] & 0x01);

			}
			if (DEBUG)
				System.out.printf("\ntestIsAlmostConvexAtVertex: " + str);
		}
		else
			assert (sombreroInitialized == true);

	}

	@Test
	public void testGetAngleABC()
	{
		initializeBaseClass();

		Point2DReadOnly nextVertex = new Point2D(0, 0);
		Point2DReadOnly previousVertex = new Point2D(0, 0);
		Point2DReadOnly vertex = new Point2D(0, 0);
		double angleABC = ConcaveHullTools.getAngleABC(nextVertex, previousVertex, vertex);
		assertEquals(angleABC, 0, EPS);

		//public static double getAngleABC(Point2DReadOnly nextVertex, Point2DReadOnly previousVertex, Point2DReadOnly vertex)
		if (DEBUG)
			System.out.printf("\ntestGetAngleABC: %f", angleABC);
	}

	@Test
	public void testGetAngleFromPreviousEdgeToNextEdge()
	{
		initializeBaseClass();

		int vertexIndex = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = sombrero;
		double angleFromPreviousEdgeToNextEdge = ConcaveHullTools.getAngleFromPreviousEdgeToNextEdge(vertexIndex, concaveHullVertices);
		assertEquals(angleFromPreviousEdgeToNextEdge, -1.707571, EPS);

		if (DEBUG)
			System.out.printf("\ntestGetAngleFromPreviousEdgeToNextEdge: %f", angleFromPreviousEdgeToNextEdge);
	}

	public static void main(String[] args)
	{
		MutationTestFacilitator.facilitateMutationTestForClass(ConcaveHullToolsTest.class, ConcaveHullToolsTest.class);
	}

}
