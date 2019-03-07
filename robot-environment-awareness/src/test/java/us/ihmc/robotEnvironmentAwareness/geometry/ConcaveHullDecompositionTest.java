package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Assert;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import static us.ihmc.commons.lists.ListWrappingIndexTools.next;
import static us.ihmc.commons.lists.ListWrappingIndexTools.removeAllExclusive;
import static us.ihmc.commons.lists.ListWrappingIndexTools.subListInclusive;
import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.findClosestIntersectionWithRay;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.lang3.mutable.MutableBoolean;


import com.sun.javafx.application.PlatformImpl;
import com.vividsolutions.jts.geom.MultiPoint;

import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.MutationTestFacilitator;
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
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;

public class ConcaveHullDecompositionTest extends ConcaveHullTestBasics
{
	private static final boolean DEBUG = true;
	private static final double EPS = 1.0e-6;

	public ConcaveHullDecompositionTest()
	{
		VISUALIZE = false;
	}

	private ConcaveHull concaveHull = null;
	private ConcaveHull convexHull = null;
	private ConcaveHullPocket pocket = null;

	@Test
	public void recursiveApproximateDecompositionConcaveHullCollectionn()
	{
		initializeBaseClass();
		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombreroCollection, depth1, convexPolygonsToPack);

		int numberOfHulls = convexPolygonsToPack.size();		
		assert(2 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionConcaveHullCollection(depth1): %d\n", numberOfHulls);

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombreroCollection, depth2, convexPolygonsToPack);

		numberOfHulls = convexPolygonsToPack.size();		
		assert(4 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionConcaveHullCollection(depth2): %d\n", numberOfHulls);
}

	@Test
	public void recursiveApproximateDecompositionConcaveHull()
	{
		initializeBaseClass();
		double depthThreshold = 0;
		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombreroHull, depth1, convexPolygonsToPack);

		int numberOfHulls = convexPolygonsToPack.size();		
		assert(2 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionConcaveHull(depth1): %d\n", numberOfHulls);

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombreroHull, depth2, convexPolygonsToPack);

		numberOfHulls = convexPolygonsToPack.size();		
		assert(4 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionConcaveHull(depth2): %d\n", numberOfHulls);
	}

	@Test
	public void recursiveApproximateDecompositionPoint2DList() throws Exception
	{
		initializeBaseClass();
		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombrero, depth1, convexPolygonsToPack);

		int numberOfHulls = convexPolygonsToPack.size();		
		assert(2 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionPoint2DList: %d\n", numberOfHulls);

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombreroHull, depth2, convexPolygonsToPack);

		numberOfHulls = convexPolygonsToPack.size();		
		assert(4 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionPoint2DList(depth2): %d\n", numberOfHulls);
}

	@Test
	public void recursiveApproximateDecompositionInternal() throws Exception
	{
		initializeBaseClass();
		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombrero, depth1, convexPolygonsToPack);

		int numberOfHulls = convexPolygonsToPack.size();		
		assert(2 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionInternal: %d\n", numberOfHulls);

		ConcaveHullDecomposition.recursiveApproximateDecomposition(sombreroHull, depth2, convexPolygonsToPack);

		numberOfHulls = convexPolygonsToPack.size();		
		assert(4 == numberOfHulls);
		if(DEBUG) System.out.printf("recursiveApproximateDecompositionInternal(depth2): %d\n", numberOfHulls);
	}
	
	
	@Test
	public void testRecursiveApproximateDecompositionInternal() throws Exception
	{

		initializeBaseClass();
		MultiPoint multiPoint = SimpleConcaveHullFactory.filterAndCreateMultiPoint(getPointcloud2D(), getLineConstraints2D(), .001);

		PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud3D);
		data.addIntersections(lineConstraints3D);
	
		ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
		parameters.setTriangulationTolerance(1.0e-5);
		parameters.setEdgeLengthThreshold(0.15);
	
		messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

		AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);

		messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

		while (output.get() == null)
			ThreadTools.sleep(100);

		ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();

		int numberOfHulls = concaveHullCollection.getNumberOfConcaveHulls();

		if(DEBUG) System.out.printf("\nConcaveHullDecompositionTest: numberOfHulls = %d ", numberOfHulls);

		assertEquals(numberOfHulls, 2);

		int hullCount = 0;

		concaveHull = null;
		convexHull = null;
		pocket = null;

		for (ConcaveHull hull : concaveHullCollection)
		{
			int num = hull.getNumberOfVertices();
			if(DEBUG) System.out.printf("\nConcaveHullTDecompositionTest: Hull %d is %s, number of vertices = %d", hullCount, hull.isHullConvex() ? "Convex" : "Concave", num);

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
		assert (pocket == null);

		pocket = concaveHull.findFirstConcaveHullPocket();
		assert (pocket != null);
		assertEquals(pocket.getMaxDepth(), 0.42956, EPS);

		int vertex = 0;
		do
		{
			pocket = concaveHull.computeConcaveHullPocket(vertex);
			if (pocket != null)
			{
				assert (vertex == 2);
				if(DEBUG) System.out.printf("\nConcaveHullDecompositionTest: pocket at vertex %d depth is %f\n", vertex, pocket.getMaxDepth());
			}
			vertex++;
		}
		while (pocket == null);
		assertEquals(pocket.getMaxDepth(), 0.42956, EPS);

		//		private static List<PlanarRegion> PlanarRegionPolygonizer.createPlanarRegion(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters, PlanarRegionSegmentationDataExporter dataExporter)
		//		PlanarRegionPolygonizer.createPlanarRegion(rawData, concaveHullFactoryParameters, polygonizerParameters, dataExporter);
		//		private static void recursiveApproximateDecompositionInternal(List<Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)

		ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
		PlanarRegion planarRegion = new PlanarRegion();

		try
		{
			planarRegion.getConcaveHull();

			if (planarRegion.getNumberOfConvexPolygons() == 0)
				return;

			CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(planarRegion);

			if (planarRegion.getConcaveHull() == null || planarRegion.getConcaveHull().length == 0)
				throw new IllegalArgumentException("Invalid planar region: missing the concave hull information.");

			List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
			double depthThreshold = 0.01;
			ConcaveHullDecomposition.recursiveApproximateDecomposition(Arrays.asList(planarRegion.getConcaveHull()), depthThreshold, decomposedPolygons);
			RigidBodyTransform transformToWorld = new RigidBodyTransform();
			planarRegion.getTransformToWorld(transformToWorld);
			planarRegion.set(transformToWorld, decomposedPolygons);
		}
		catch (Exception e)
		{
			System.out.println("\ntestRecursiveApproximateDecompositionInternal: exception = " + e);
		}
	}

	//public static void recursiveApproximateDecomposition(ConcaveHullCollection concaveHullCollection, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
	//public static void recursiveApproximateDecomposition(ConcaveHull concaveHull, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
	//public static void recursiveApproximateDecomposition(List<? extends Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
	//private static void recursiveApproximateDecompositionInternal(List<Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)

	
	public static void main(String[] args)
	{
		MutationTestFacilitator.facilitateMutationTestForClass(ConcaveHullDecompositionTest.class, ConcaveHullDecompositionTest.class);
	}

}
