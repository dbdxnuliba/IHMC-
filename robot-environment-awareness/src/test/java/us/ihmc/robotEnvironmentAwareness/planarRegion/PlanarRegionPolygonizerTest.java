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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
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

public class PlanarRegionPolygonizerTest {
	private static boolean VISUALIZE = false;

	private Messager messager;
	private MutableBoolean uiIsGoingDown = new MutableBoolean(false);

	@Before
	public void setup() throws Exception {
		uiIsGoingDown.setFalse();

		if (VISUALIZE) {
			SharedMemoryJavaFXMessager jfxMessager = new SharedMemoryJavaFXMessager(
					PolygonizerVisualizerUI.getMessagerAPI());
			messager = jfxMessager;
			createVisualizer(jfxMessager);
		} else {
			messager = new SharedMemoryMessager(PolygonizerVisualizerUI.getMessagerAPI());
			messager.startMessager();
			new PolygonizerManager(messager);
		}
	}

	@SuppressWarnings("restriction")
	private void createVisualizer(JavaFXMessager messager) {
		AtomicReference<PolygonizerVisualizerUI> ui = new AtomicReference<>(null);

		PlatformImpl.startup(() -> {
			try {
				Stage primaryStage = new Stage();
				primaryStage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, event -> uiIsGoingDown.setTrue());

				ui.set(new PolygonizerVisualizerUI(messager, primaryStage));
				ui.get().show();
			} catch (Exception e) {
				e.printStackTrace();
			}
		});

		while (ui.get() == null)
			ThreadTools.sleep(200);
	}

	@After
	public void tearDown() {
		if (VISUALIZE) {
			while (!uiIsGoingDown.booleanValue())
				ThreadTools.sleep(100);
		}
	}

//	public PlanarRegionsList createPlanarRegionsList(List<PlanarRegionSegmentationRawData> rawData,
//			ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters) {
//		PlanarRegionsList ret = null;
//		return ret;
//	}
//
//	public PlanarRegionsList createPlanarRegionsList(List<PlanarRegionSegmentationRawData> rawData,
//			ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
//			PlanarRegionSegmentationDataExporter dataExporter) {
//		PlanarRegionsList ret = null;
//		return ret;
//	}
//
//	private List<PlanarRegion> createPlanarRegions(List<PlanarRegionSegmentationRawData> rawData,
//			ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
//			PlanarRegionSegmentationDataExporter dataExporter) {
//		List<PlanarRegion> ret = null;
//		return ret;
//	}
//
//	private List<PlanarRegion> createPlanarRegion(PlanarRegionSegmentationRawData rawData,
//			ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
//			PlanarRegionSegmentationDataExporter dataExporter) {
//		List<PlanarRegion> ret = null;
//		return ret;
//	}

	List<PlanarRegionSegmentationRawData> rawDataList = new ArrayList();
	PlanarRegionSegmentationRawData rawData = null;
	ConcaveHullFactoryParameters concaveHullFactoryParameters = null;
	PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
	PlanarRegionSegmentationDataExporter dataExporter = null;

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
	public final void testPlanarRegionPolygonizer() {
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

		PlanarRegionPolygonizer.createPlanarRegionsList(rawDataList, concaveHullFactoryParameters,
				polygonizerParameters);

		// PlanarRegionPolygonizer.createPlanarRegionsList(rawDataList,
		// concaveHullFactoryParameters,
		// polygonizerParameters, dataExporter);

		AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
		// System.out.println("PlanarRegionPolygonizerTest: output: " + output.get().toString());

		messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(rawData));

		while (output.get() == null)
			ThreadTools.sleep(100);

		ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult()
				.getConcaveHullCollection();
		System.out.println("PlanarRegionPolygonizerTest: concaveHullCollection: " + concaveHullCollection.getConcaveHulls().toString());

		assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());

		Object[] hull = concaveHullCollection.getConcaveHulls().toArray();
		//for (int i = 0; i < hull.length; i++)
		//	System.out.println("PlanarRegionPolygonizerTest: hull[" + i + "]: " + hull[i].toString());
		
		//ConcaveHullPocket  chp =  findFirstConcaveHullPocket(concaveHullVertices);

	}

}
