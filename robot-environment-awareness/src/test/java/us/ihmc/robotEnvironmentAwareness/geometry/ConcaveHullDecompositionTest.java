package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
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
	public ConcaveHullDecompositionTest()
	{
		VISUALIZE = false;
	}

	private ConcaveHull concaveHull = null;
	private ConcaveHull convexHull = null;
	private ConcaveHullPocket pocket = null;

	@Test(timeout = 30000)
	public void recursiveApproximateDecompositionConcaveHullCollectionn() throws Exception
	{
		initializeBasics();
		ConcaveHullCollection concaveHullCollection = new ConcaveHullCollection();
		double depthThreshold = 0;
		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();

		ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullCollection, depthThreshold, convexPolygonsToPack);
		//		  public static void recursiveApproximateDecomposition(ConcaveHullCollection concaveHullCollection, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
	}

//	@Test(timeout = 30000)
//	public void recursiveApproximateDecompositionConcaveHull() throws Exception
//	{
//		initializeBasics();
//		ConcaveHull concaveHull = new ConcaveHull(concaveHull);
//		double depthThreshold = 0;
//		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();
//
//		ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, depthThreshold, convexPolygonsToPack);
//		//   public static void recursiveApproximateDecomposition(ConcaveHull concaveHull, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
//
//	}

	@Test(timeout = 30000)
	public void recursiveApproximateDecompositionPoint2DList() throws Exception
	{
		initializeBasics();
		List<? extends Point2DReadOnly> concaveHullVertices = new ArrayList<>();
		double depthThreshold = 0;
		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();

		ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVertices, depthThreshold, convexPolygonsToPack);
		//   public static void recursiveApproximateDecomposition(List<? extends Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
	}

//	@Test(timeout = 30000)
//	public void recursiveApproximateDecompositionInternal() throws Exception
//	{
// 	initializeBasics();
//		List<? extends Point2DReadOnly> concaveHullVertices = new ArrayList<>();
//		double depthThreshold = 0;
//		List<ConvexPolygon2D> convexPolygonsToPack = new ArrayList<>();
//
//		ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVertices, depthThreshold, convexPolygonsToPack);
//	//   private static void recursiveApproximateDecompositionInternal(List<Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
//	}
	
	
	@Test(timeout = 30000)
	public void testRecursiveApproximateDecompositionInternal() throws Exception
	{

		initializeBasics();
		MultiPoint multiPoint = SimpleConcaveHullFactory.filterAndCreateMultiPoint(getPointcloud2D(), getLineConstraints2D(), .001);

		PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud3D);
		data.addIntersections(lineConstraints3D);
		// System.out.println("ConcaveHullDecompositionTest: data: " + data.toString());

		ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
		parameters.setTriangulationTolerance(1.0e-5);
		parameters.setEdgeLengthThreshold(0.15);
		// System.out.println("ConcaveHullDecompositionTest: parameters: " + parameters.toString());

		messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

		AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
		// System.out.println("ConcaveHullDecompositionTest: output: " + output.get().toString());

		messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

		while (output.get() == null)
			ThreadTools.sleep(100);

		ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
		//System.out.println("ConcaveHullDecompositionTest: concaveHullCollection: " + concaveHullCollection.getConcaveHulls());

		int numberOfHulls = concaveHullCollection.getNumberOfConcaveHulls();

		//System.out.printf("ConcaveHullDecompositionTest: numberOfHulls = %d ", numberOfHulls);

		assertEquals(numberOfHulls, 2);

		int hullCount = 0;

		concaveHull = null;
		convexHull = null;
		pocket = null;

		for (ConcaveHull hull : concaveHullCollection)
		{
			int num = hull.getNumberOfVertices();
			//System.out.printf("\nConcaveHullTDecompositionTest: Hull %d is %s, number of vertices = %d", hullCount, hull.isHullConvex() ? "Convex" : "Concave", num);

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
				//System.out.printf("ConcaveHullDecompositionTest: pocket at vertex %d depth is %f", vertex, pocket.getMaxDepth());
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

			if (planarRegion.getNumberOfConvexPolygons() > 0)
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

}
