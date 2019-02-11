package us.ihmc.robotEnvironmentAwareness.planarRegion;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.sun.javafx.application.PlatformImpl;

import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPocket;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PolygonizerToolsTest
{
	private static boolean VISUALIZE = false;

	private Messager messager;
	private MutableBoolean uiIsGoingDown = new MutableBoolean(false);

	@Before
	public void setup() throws Exception
	{
		uiIsGoingDown.setFalse();

		if (VISUALIZE)
		{
			SharedMemoryJavaFXMessager jfxMessager = new SharedMemoryJavaFXMessager(PolygonizerVisualizerUI.getMessagerAPI());
			messager = jfxMessager;
			createVisualizer(jfxMessager);
		}
		else
		{
			messager = new SharedMemoryMessager(PolygonizerVisualizerUI.getMessagerAPI());
			messager.startMessager();
			new PolygonizerManager(messager);
		}
	}

	@SuppressWarnings("restriction")
	private void createVisualizer(JavaFXMessager messager)
	{
		AtomicReference<PolygonizerVisualizerUI> ui = new AtomicReference<>(null);

		PlatformImpl.startup(() -> {
			try
			{
				Stage primaryStage = new Stage();
				primaryStage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, event -> uiIsGoingDown.setTrue());

				ui.set(new PolygonizerVisualizerUI(messager, primaryStage));
				ui.get().show();
			}
			catch (Exception e)
			{
				e.printStackTrace();
			}
		});

		while (ui.get() == null)
			ThreadTools.sleep(200);
	}

	@After
	public void tearDown()
	{
		if (VISUALIZE)
		{
			while (!uiIsGoingDown.booleanValue())
				ThreadTools.sleep(100);
		}
	}

	List<PlanarRegionSegmentationRawData> rawDataList = new ArrayList();
	PlanarRegionSegmentationRawData rawData = null;
	ConcaveHullFactoryParameters concaveHullFactoryParameters = null;
	PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
	PlanarRegionSegmentationDataExporter dataExporter = null;

	
//	public class Orientation3D implements Orientation3DReadOnly {
//	    public void Eat(String food_name) {
//	        System.out.printf(food_name);
//	    }
	

	@Test(timeout = 30000)
	public void testToPointsInPlaneVector()
	{
		List<? extends Point3DReadOnly> pointsToTransform = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		List<Point2D> pointsInPlane = PolygonizerTools.toPointsInPlane(pointsToTransform, planeOrigin, planeNormal);
		//public static List<Point2D> toPointsInPlane(List<? extends Point3DReadOnly> pointsToTransform, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal)
		System.out.println("testToPointsInPlaneVector: pointsInPlane = " + pointsInPlane.size());
	}

	@Test(timeout = 30000)
	public void testToPointsInPlaneOrientation()
	{
		List<? extends Point3DReadOnly> pointsToTransform = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();

		List<Point2D> pointsInPlane = PolygonizerTools.toPointsInPlane(pointsToTransform, planeOrigin, planeOrientation);
		//public static List<Point2D> toPointsInPlane(List<? extends Point3DReadOnly> pointsToTransform, Point3DReadOnly planeOrigin,
		//                                            Orientation3DReadOnly planeOrientation)
		System.out.println("testToPointsInPlaneOrientation: pointsInPlane = " + pointsInPlane.size());
	}

	@Test(timeout = 30000)
	public void testToPointInPlaneOrientation()
	{
		Point3DReadOnly pointToTransform = new Point3D();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		Point2D pointInPlane = PolygonizerTools.toPointInPlane(pointToTransform, planeOrigin, planeOrientation);
				//public static Point2D toPointInPlane(Point3DReadOnly pointToTransform, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation) 
		System.out.println("testToPointInPlaneOrientation: pointInPlane = " + pointInPlane);
	}

	@Test(timeout = 30000)
	public void testToLineSegmentsInPlaneVector3D()
	{
		List<? extends LineSegment3DReadOnly> lineSegmentsToTransform = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		List<LineSegment2D> lineSegmentsInPlane = PolygonizerTools.toLineSegmentsInPlane(lineSegmentsToTransform, planeOrigin, planeNormal);
		//public static List<LineSegment2D> toLineSegmentsInPlane(List<? extends LineSegment3DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
		//                                                        Vector3DReadOnly planeNormal)
		System.out.println("testToLineSegmentsInPlaneVector3D: lineSegmentsInPlane = " + lineSegmentsInPlane.size());
	}

	@Test(timeout = 30000)
	public void testToLineSegmentsInPlaneOrientation()
	{
		List<? extends LineSegment3DReadOnly> lineSegmentsToTransform = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		List<LineSegment2D> lineSegmentsInPlane = PolygonizerTools.toLineSegmentsInPlane(lineSegmentsToTransform, planeOrigin, planeOrientation);
		//public static List<LineSegment2D> toLineSegmentsInPlane(List<? extends LineSegment3DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
		//                                                        Orientation3DReadOnly planeOrientation)
		System.out.println("testToLineSegmentsInPlaneOrientation: lineSegmentsInPlane = " + lineSegmentsInPlane.size());
	}

	@Test(timeout = 30000)
	public void testToLineSegmentInPlaneVector3D()
	{
		LineSegment3DReadOnly lineSegmentToTransform = new LineSegment3D();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		LineSegment2D lineSegmentInPlane = PolygonizerTools.toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, planeNormal);
		//public static LineSegment2D toLineSegmentInPlane(LineSegment3DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal) 
		System.out.println("testToLineSegmentInPlaneVector3D: lineSegmentInPlane = " + lineSegmentInPlane);
	}

	@Test(timeout = 30000)
	public void testToLineSegmentInPlaneOreintation3D()
	{
		LineSegment3DReadOnly lineSegmentToTransform = new LineSegment3D();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		LineSegment2D lineSegmentInPlane = PolygonizerTools.toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, planeOrientation);
		//public static LineSegment2D toLineSegmentInPlane(LineSegment3DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin,
		//                                                 Orientation3DReadOnly planeOrientation) 
		System.out.println("testToLineSegmentInPlaneOreintation3D: lineSegmentInPlane = " + lineSegmentInPlane);
	}

	@Test(timeout = 30000)
	public void testToPointInPlane()
	{
		double xToTransform = 0;
		double yToTransform = 0;
		double zToTransform = 0;
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		Point2D pointInPlane = PolygonizerTools.toPointInPlane(xToTransform, yToTransform, zToTransform, planeOrigin, planeOrientation);
		//public static Point2D toPointInPlane(double xToTransform, double yToTransform, double zToTransform, Point3DReadOnly planeOrigin,
		//                                     Orientation3DReadOnly planeOrientation) 
		System.out.println("testToPointInPlane: pointInPlane = " + pointInPlane);
	}

	@Test(timeout = 30000)
	public void testToPointsInWorldVector3D()
	{
		List<? extends Point2DReadOnly> pointsInPlane = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		List<Point3D> pointsInWorld = PolygonizerTools.toPointsInWorld(pointsInPlane, planeOrigin, planeNormal);
		//public static List<Point3D> toPointsInWorld(List<? extends Point2DReadOnly> pointsInPlane, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal)
		System.out.println("testToPointsInWorldVector3D: pointsInWorld = " + pointsInWorld.size());
	}

	@Test(timeout = 30000)
	public void testToPointsInWorldOrientation3D()
	{
		List<? extends Point2DReadOnly> pointsInPlane = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		List<Point3D> pointsInWorld = PolygonizerTools.toPointsInWorld(pointsInPlane, planeOrigin, planeOrientation);
		//public static List<Point3D> toPointsInWorld(List<? extends Point2DReadOnly> pointsInPlane, Point3DReadOnly planeOrigin,
		//                                            Orientation3DReadOnly planeOrientation) 
		System.out.println("testToPointsInWorldOrientation3D: pointsInWorld = " + pointsInWorld.size());
	}

	@Test(timeout = 30000)
	public void testToPointInWorldOrientation3D()
	{
		Point2DReadOnly point2dReadOnly = new Point2D();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		Point3D pointInWorld = PolygonizerTools.toPointInWorld(point2dReadOnly, planeOrigin, planeOrientation);
		//public static Point3D toPointInWorld(Point2DReadOnly point2dReadOnly, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation)
		System.out.println("testToPointInWorldOrientation3D: pointInWorld = " + pointInWorld);
	}

	@Test(timeout = 30000)
	public void testToLineSegmentsInWorldVector3D()
	{
		List<? extends LineSegment2DReadOnly> lineSegmentsToTransform = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		List<LineSegment3D> lineSegmentsInWorld = PolygonizerTools.toLineSegmentsInWorld(lineSegmentsToTransform, planeOrigin, planeNormal);
		//public static List<LineSegment3D> toLineSegmentsInWorld(List<? extends LineSegment2DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
		//                                                        Vector3DReadOnly planeNormal) 
		System.out.println("testToLineSegmentsInWorldVector3D: lineSegmentsInWorld = " + lineSegmentsInWorld.size());
	}

	@Test(timeout = 30000)
	public void testtoLineSegmentsInWorldOrientation3D()
	{
		List<? extends LineSegment2DReadOnly> lineSegmentsToTransform = new ArrayList<>();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;

		List<LineSegment3D> lineSegmentsInWorld = PolygonizerTools.toLineSegmentsInWorld(lineSegmentsToTransform, planeOrigin, planeOrientation);
		//public static List<LineSegment3D> toLineSegmentsInWorld(List<? extends LineSegment2DReadOnly> lineSegmentsToTransform, Point3DReadOnly planeOrigin,
		//                                                        Orientation3DReadOnly planeOrientation) 
		System.out.println("testtoLineSegmentsInWorldOrientation3D: lineSegmentsInWorld  = " + lineSegmentsInWorld.size());
	}

	@Test(timeout = 30000)
	public void testToLineSegmentInWorld()
	{
		LineSegment2DReadOnly lineSegmentToTransform = new LineSegment2D();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();
		LineSegment3D lineSegmentInWorld = PolygonizerTools.toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, planeNormal);
		//public static LineSegment3D toLineSegmentInWorld(LineSegment2DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin, Vector3DReadOnly planeNormal) 
		System.out.println("testToLineSegmentInWorld: lineSegmentsInWorld  = " + lineSegmentInWorld);
	}

	@Test(timeout = 30000)
	public void tesToLineSegmentInWorld()
	{
		LineSegment2DReadOnly lineSegmentToTransform = new LineSegment2D();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;
		LineSegment3D lineSegmentInWorld = PolygonizerTools.toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, planeOrientation);
		//public static LineSegment3D toLineSegmentInWorld(LineSegment2DReadOnly lineSegmentToTransform, Point3DReadOnly planeOrigin,
		//                                                Orientation3DReadOnly planeOrientation) 
		System.out.println("tesToLineSegmentInWorld: lineSegmentInWorld  = " + lineSegmentInWorld);
	}

	@Test(timeout = 30000)
	public void tesToPointInWorld()
	{
		double xToTransform = 0;
		double yToTransform = 0;
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();;
		Point3D pointInWorld = PolygonizerTools.toPointInWorld(xToTransform, yToTransform, planeOrigin, planeOrientation);
		//public static Point3D toPointInWorld(double xToTransform, double yToTransform, Point3DReadOnly planeOrigin, Orientation3DReadOnly planeOrientation)
		System.out.println("tesToPointInWorld: pointInWorld = " + pointInWorld);
	}

	@Test(timeout = 30000)
	public void testGetQuaternionFromZUpToVector()
	{
		Vector3DReadOnly normal = new Vector3D();
		Quaternion quaternionFromZUpToVector = PolygonizerTools.getQuaternionFromZUpToVector(normal);
		//public static Quaternion getQuaternionFromZUpToVector(Vector3DReadOnly normal)
		System.out.println("testGetQuaternionFromZUpToVector: quaternionFromZUpToVector  = " + quaternionFromZUpToVector);
	}

	@Test(timeout = 30000)
	public void testComputeEllipsoidVolumeRadii()
	{
		Vector3DReadOnly radii = new Vector3D();
		double ellipsoidVolume = PolygonizerTools.computeEllipsoidVolume(radii);
		//public static double computeEllipsoidVolume(Vector3DReadOnly radii)
		System.out.println("testComputeEllipsoidVolumeRadii: ellipsoidVolume = " + ellipsoidVolume);
	}

	@Test(timeout = 30000)
	public void testComputeEllipsoidVolumeXYZ()
	{
		double xRadius = 0;
		double yRadius = 0;
		double zRadius = 0;
		double ellipsoidVolume = PolygonizerTools.computeEllipsoidVolume(xRadius, yRadius, zRadius);
		//public static double computeEllipsoidVolume(double xRadius, double yRadius, double zRadius)
		System.out.println("testComputeEllipsoidVolumeXYZ: ellipsoidVolume = " + ellipsoidVolume);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
	public final void testPlanarRegionPolygonizer()
	{
		List<Point3D> pointcloud = new ArrayList<>();
		pointcloud.add(new Point3D(0.5, -0.1, 0.0));
		pointcloud.add(new Point3D(0.5, 0.1, 0.0));
		pointcloud.add(new Point3D(1.5, 0.1, 0.0));
		pointcloud.add(new Point3D(1.5, -0.1, 0.0));

		// pointcloud.add(new Point3D(0.5, -0.1, 0.0));
		pointcloud.add(new Point3D(1.4, 1.0, 0.0));
		pointcloud.add(new Point3D(1.5, 1.0, 0.0));
		pointcloud.add(new Point3D(1.4, 0.10, 0.0));
		System.out.println("\npointcloud: " + pointcloud.toString());

		List<LineSegment3D> lineConstraints = new ArrayList<>();
		lineConstraints.add(new LineSegment3D(0.0, -0.5, 0.0, 0.0, 0.5, 0.0));
		lineConstraints.add(new LineSegment3D(2.0, -0.5, 0.0, 2.0, 0.5, 0.0));
		lineConstraints.add(new LineSegment3D(0.0, 0.5, 0.0, 2.0, 0.5, 0.0));
		lineConstraints.add(new LineSegment3D(0.0, -0.5, 0.0, 2.0, -0.5, 0.0));
		System.out.println("\nlineConstraints: " + lineConstraints.toString());

		// PlanarRegionSegmentationRawData
		rawData = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud);
		rawData.addIntersections(lineConstraints);
		System.out.println("PlanarRegionPolygonizerTest: rawData: " + rawData.toString());

		rawDataList.add(rawData);
		System.out.println("PlanarRegionPolygonizerTest: rawDataList: " + rawDataList.toString());

		concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
		concaveHullFactoryParameters.setTriangulationTolerance(1.0e-5);
		concaveHullFactoryParameters.setEdgeLengthThreshold(0.15);
		System.out.println("\nparameters: " + concaveHullFactoryParameters.toString());

		messager.submitMessage(Polygonizer.PolygonizerParameters, concaveHullFactoryParameters);

		PlanarRegionPolygonizer.createPlanarRegionsList(rawDataList, concaveHullFactoryParameters, polygonizerParameters);

		// PlanarRegionPolygonizer.createPlanarRegionsList(rawDataList,
		// concaveHullFactoryParameters,
		// polygonizerParameters, dataExporter);

		AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
		// System.out.println("PlanarRegionPolygonizerTest: output: " + output.get().toString());

		messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(rawData));

		while (output.get() == null)
			ThreadTools.sleep(100);

		ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
		System.out.println("PlanarRegionPolygonizerTest: concaveHullCollection: " + concaveHullCollection.getConcaveHulls().toString());

		assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());

		Object[] hull = concaveHullCollection.getConcaveHulls().toArray();
		//for (int i = 0; i < hull.length; i++)
		//	System.out.println("PlanarRegionPolygonizerTest: hull[" + i + "]: " + hull[i].toString());

		//ConcaveHullPocket  chp =  findFirstConcaveHullPocket(concaveHullVertices);

	}

}
