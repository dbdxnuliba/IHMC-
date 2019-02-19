package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.commons.lists.ListWrappingIndexTools.getNext;
import static us.ihmc.commons.lists.ListWrappingIndexTools.getPrevious;
import static us.ihmc.commons.lists.ListWrappingIndexTools.next;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.sun.javafx.application.PlatformImpl;
import com.vividsolutions.jts.geom.MultiPoint;

import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;

public class ConcaveHullToolsTest extends ConcaveHullTestBasics
{
	public ConcaveHullToolsTest()
	{
		VISUALIZE = false;
	}
	

	@Test(timeout = 30000)
	public void testPointCloudWithSurroundingLineConstraints() throws Exception
	{
		initializeBasics();

		MultiPoint multiPoint = SimpleConcaveHullFactory.filterAndCreateMultiPoint(getPointcloud2D(), getLineConstraints2D(), .001);

		PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud3D);
		data.addIntersections(lineConstraints3D);
		// System.out.println("ConcaveHullToolsTest: data: " + data.toString());

		ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
		parameters.setTriangulationTolerance(1.0e-5);
		parameters.setEdgeLengthThreshold(0.15);
		// System.out.println("ConcaveHullToolsTest: parameters: " + parameters.toString());

		messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

		AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
		// System.out.println("ConcaveHullToolsTest: output: " + output.get().toString());

		messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

		while (output.get() == null)
			ThreadTools.sleep(100);

		ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
		//System.out.println("ConcaveHullToolsTest: concaveHullCollection: " + concaveHullCollection.getConcaveHulls());

		int numberOfHulls = concaveHullCollection.getNumberOfConcaveHulls();

		//System.out.printf("ConcaveHullToolsTest: numberOfHulls = %d ", numberOfHulls);

		assertEquals(numberOfHulls, 2);

		int hullCount = 0;
		ConcaveHull concaveHull = null; 
		ConcaveHull convexHull = null;
		ConcaveHullPocket pocket = null;
		
		for (ConcaveHull hull : concaveHullCollection)
		{
			int num = hull.getNumberOfVertices();
			//System.out.printf("\nConcaveHullToolsTest: Hull %d is %s, number of vertices = %d", hullCount, hull.isHullConvex() ? "Convex" : "Concave", num);

			if (hullCount == 0)
			{
				assert (hull.isHullConvex() == false);
				assert (num == 24);				
				concaveHull = hull;								
			}
			if (hullCount == 1)
			{
				assert (hull.isHullConvex() == true);
				assert (num == 4);				
				convexHull = hull;								
			}
			hullCount++;
		}
		
		pocket = convexHull.findFirstConcaveHullPocket();
		assert(pocket == null);
		
		pocket = concaveHull.findFirstConcaveHullPocket();
		assert(pocket != null);
		assertEquals(pocket.getMaxDepth(), 0.42956, EPS);
		
		int vertex=0;
		do
		{
			pocket = concaveHull.computeConcaveHullPocket(vertex);
			if( pocket != null)
			{
				assert(vertex==2);
				assertEquals(pocket.getMaxDepth(), 0.429561, EPS);
				//System.out.printf("ConcaveHullToolsTest: pocket at vertex %d depth is %f", vertex, pocket.getMaxDepth());
			}
			vertex++;
		} while(pocket == null);
		assertEquals(pocket.getMaxDepth(), 0.42956, EPS);		
	}


	@Test(timeout = 30000)
	public void testEnsureCounterClockwiseOrdering() throws Exception
	{
		initializeBasics();

		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		ConcaveHullTools.ensureCounterClockwiseOrdering(concaveHullVertices);
		
		//public static void ensureCounterClockwiseOrdering(List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.println("\ntestEnsureCounterClockwiseOrdering: ");
	}


	@Test(timeout = 30000)
	public void testComputePerimeter() throws Exception
	{
		initializeBasics();

		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		double perimeter = ConcaveHullTools.computePerimeter(concaveHullVertices);
		assertEquals(perimeter, 43.355019, EPS);
	
	//public static double computePerimeter(List<? extends Point2DReadOnly> concaveHullVertices)
	//System.out.printf("\ntestComputePerimeter: %f", perimeter);
	}

	@Test(timeout = 30000)
	public void testRemoveSuccessiveDuplicateVertices() throws Exception
	{
		initializeBasics();

		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		int successiveDuplicateVertices = ConcaveHullTools.removeSuccessiveDuplicateVertices(concaveHullVertices);
		assert(successiveDuplicateVertices == 0);

		//public static int removeSuccessiveDuplicateVertices(List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.printf("\ntestRemoveSuccessiveDuplicateVertices: %d", successiveDuplicateVertices);
	}


	   
	@Test(timeout = 30000)
	public void testComputeConcaveHullPocket() throws Exception
	{
		initializeBasics();

		int concaveVertexIndex = 0;
		ConcaveHullPocket pocketToPack = new ConcaveHullPocket();
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		boolean concaveHullPocket = ConcaveHullTools.computeConcaveHullPocket(concaveVertexIndex, pocketToPack, concaveHullVertices);
		assert(concaveHullPocket == false);
		
		//public static boolean computeConcaveHullPocket(int concaveVertexIndex, ConcaveHullPocket pocketToPack, List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.printf("\ntestComputeConcaveHullPocket: %b", concaveHullPocket);
   }

   	
   @Test(timeout = 30000)
	public void testFindConcaveHullPockets() throws Exception
	{
		initializeBasics();

		double depthThreshold = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		Set<ConcaveHullPocket> concaveHullPockets = ConcaveHullTools.findConcaveHullPockets(concaveHullVertices, depthThreshold);	

		assert(concaveHullPockets != null);
		assert(concaveHullPockets.size() == 6) ;
		
		//public static Set<ConcaveHullPocket> findConcaveHullPockets(List<? extends Point2DReadOnly> concaveHullVertices, double depthThreshold)
		//System.out.println("\ntestFindConcaveHullPockets: " + concaveHullPockets.size());
	}
	 
		  
	@Test(timeout = 30000)
	public void testFindClosestIntersectionWithRay() throws Exception
	{
		initializeBasics();

		Point2DReadOnly rayOrigin = new Point2D(0,0);
		Vector2DReadOnly rayDirection = new Vector2D(0,0);
		int startSearchIndex = 0, endSearchIndex = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		Point2DBasics intersectionToPack = (Point2DBasics) new Point2D(0,0);
		
		int closestIntersectionWithRay = ConcaveHullTools.findClosestIntersectionWithRay(rayOrigin, rayDirection, startSearchIndex, endSearchIndex, 
		                                                                concaveHullVertices, intersectionToPack);
		assert(closestIntersectionWithRay == -1);

		//public static int findClosestIntersectionWithRay(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, int startSearchIndex, int endSearchIndex,
		//List<? extends Point2DReadOnly> concaveHullVertices, Point2DBasics intersectionToPack)
		//System.out.printf("\ntestFindClosestIntersectionWithRay: %d", closestIntersectionWithRay);
	}
	
	
	@Test(timeout = 30000)
	public void testFirstConcaveHullPocketInefficient() throws Exception
	{
		initializeBasics();

		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 	
		ConcaveHullPocket firstConcaveHullPocketInefficient = ConcaveHullTools.findFirstConcaveHullPocketInefficient(concaveHullVertices);
		assert(firstConcaveHullPocketInefficient != null);
		if(firstConcaveHullPocketInefficient != null)
		{
			assert(firstConcaveHullPocketInefficient.getDeepestVertexIndex() == 18);
			assertEquals(firstConcaveHullPocketInefficient.getMaxDepth(), 1.27138, EPS);
		}
		
		//public static ConcaveHullPocket findFirstConcaveHullPocketInefficient(List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.println("\ntestFirstConcaveHullPocketInefficient: getDeepestVertexIndex() = " + firstConcaveHullPocketInefficient.getDeepestVertexIndex());
		//System.out.println("\ntestFirstConcaveHullPocketInefficient: getMaxDepth() = " + firstConcaveHullPocketInefficient.getMaxDepth());
	}
	  	
	
	@Test(timeout = 30000)
	public void testFindInnerClosestEdgeToVertex() throws Exception
	{
		initializeBasics();

		int vertexIndex = 0, deadIndexRegion = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		Point2DBasics closestPointToPack = new Point2D(0,0);		
		int innerClosestEdgeToVertex = ConcaveHullTools.findInnerClosestEdgeToVertex(vertexIndex, deadIndexRegion, 
		                                                                           concaveHullVertices, closestPointToPack);
		assert( innerClosestEdgeToVertex == 1);
		
		//public static int findInnerClosestEdgeToVertex(int vertexIndex, int deadIndexRegion, 
		//List<? extends Point2DReadOnly> concaveHullVertices, Point2DBasics closestPointToPack)
		//System.out.printf("\ntestFindInnerClosestEdgeToVertex: %d", innerClosestEdgeToVertex);
	}
	
   
	@Test(timeout = 30000)
	public void testIsVertexPreventingKink() throws Exception
	{
		initializeBasics();

		int vertexIndex = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D;
		boolean isVertexPreventingKink =  ConcaveHullTools.isVertexPreventingKink(vertexIndex, concaveHullVertices);
		assert(isVertexPreventingKink == true);
		
		//public static boolean isVertexPreventingKink(int vertexIndex, List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.printf("\ntestIsVertexPreventingKink: %b", isVertexPreventingKink);
	}
 
	
	@Test(timeout = 30000)
	public void testIsAlmostConvexAtVertex() throws Exception
	{
		initializeBasics();

		int vertexIndex = 0;
		double angleTolerance = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		boolean isAlmostConvexAtVertex = ConcaveHullTools.isAlmostConvexAtVertex(vertexIndex, angleTolerance, concaveHullVertices);
		assert(isAlmostConvexAtVertex == true);
		
		//public static boolean isAlmostConvexAtVertex(int vertexIndex, double angleTolerance, List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.printf("\nisAlmostConvexAtVertex: %b", isAlmostConvexAtVertex);
	}
	
	
	@Test(timeout = 30000)
	public void testGetAngleABC() throws Exception
	{
		initializeBasics();

		Point2DReadOnly nextVertex = new Point2D(0,0);
		Point2DReadOnly previousVertex = new Point2D(0,0);
		Point2DReadOnly vertex = new Point2D(0,0);
		double angleABC = ConcaveHullTools.getAngleABC(nextVertex, previousVertex, vertex);
		assertEquals(angleABC, 0, EPS);
		
		//public static double getAngleABC(Point2DReadOnly nextVertex, Point2DReadOnly previousVertex, Point2DReadOnly vertex)
		//System.out.printf("\ntestGetAngleABC: %f", angleABC);
	}
	
	
	@Test(timeout = 30000)
	public void testGetAngleFromPreviousEdgeToNextEdge() throws Exception
	{
		initializeBasics();

		int vertexIndex = 0;
		List<? extends Point2DReadOnly> concaveHullVertices = pointcloud2D; 
		double angleFromPreviousEdgeToNextEdge = ConcaveHullTools.getAngleFromPreviousEdgeToNextEdge(vertexIndex, concaveHullVertices);
		assertEquals(angleFromPreviousEdgeToNextEdge, -2.189926, EPS);
		
		//public static double getAngleFromPreviousEdgeToNextEdge(int vertexIndex, List<? extends Point2DReadOnly> concaveHullVertices)
		//System.out.printf("\ntestGetAngleFromPreviousEdgeToNextEdge: %f", angleFromPreviousEdgeToNextEdge);
	}
		
}
