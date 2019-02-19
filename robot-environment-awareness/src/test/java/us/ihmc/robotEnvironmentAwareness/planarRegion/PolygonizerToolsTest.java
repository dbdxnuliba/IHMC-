package us.ihmc.robotEnvironmentAwareness.planarRegion;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTestBasics;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;

public class PolygonizerToolsTest extends ConcaveHullTestBasics
{
	public PolygonizerToolsTest()
	{
		VISUALIZE = false;
	}

	List<PlanarRegionSegmentationRawData> rawDataList = new ArrayList();
	PlanarRegionSegmentationRawData rawData = null;
	ConcaveHullFactoryParameters concaveHullFactoryParameters = null;
	PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
	PlanarRegionSegmentationDataExporter dataExporter = null;

	@Test(timeout = 30000)
	public void testToPointsInPlaneVector()
	{
		initializeBasics();
		List<? extends Point3DReadOnly> pointsToTransform = getPointcloud3D();
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Vector3DReadOnly planeNormal = new Vector3D(0, 1, 0);

		try
		{
			List<Point2D> pointsInPlane = PolygonizerTools.toPointsInPlane(pointsToTransform, planeOrigin, planeNormal);

			assertEquals(pointsInPlane.size(), 74);
			//System.out.println("testToPointsInPlaneVector: pointsInPlane = " + pointsInPlane.size());
		}
		catch (Exception e)
		{
			System.out.println("testToPointsInPlaneVector: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToPointsInPlaneOrientation()
	{
		initializeBasics();
		List<? extends Point3DReadOnly> pointsToTransform = getPointcloud3D();
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);

		try
		{
			List<Point2D> pointsInPlane = PolygonizerTools.toPointsInPlane(pointsToTransform, planeOrigin, planeOrientation);

			assertEquals(pointsInPlane.size(), 74);
			//System.out.println("testToPointsInPlaneOrientation: pointsInPlane = " + pointsInPlane.size());
		}
		catch (Exception e)
		{
			System.out.println("testToPointsInPlaneOrientation: exception = " + e);
		}
	}

	@Test(timeout = 30000)
	public void testToPointInPlaneOrientation()
	{
		initializeBasics();
		Point3DReadOnly pointToTransform = new Point3D(1, 1, 1);
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);

		try
		{
			Point2D pointInPlane = PolygonizerTools.toPointInPlane(pointToTransform, planeOrigin, planeOrientation);

			assertEquals(pointInPlane.getX(), -1.0, EPS);
			assertEquals(pointInPlane.getY(), 1.0, EPS);
			//System.out.println("testToPointInPlaneOrientation: pointInPlane = " + pointInPlane);
		}
		catch (Exception e)
		{
			System.out.println("testToPointInPlaneOrientation: exception = " + e);
		}
	}

	@Test(timeout = 30000)
	public void testToLineSegmentsInPlaneVector3D()
	{
		initializeBasics();
		List<? extends LineSegment3DReadOnly> lineSegmentsToTransform = getLineConstraints3D();
		Point3DReadOnly planeOrigin = new Point3D(0.0, 0.0, 0.0);
		Vector3DReadOnly planeNormal = new Vector3D(0.0, 1.0, 0.0);

		try
		{
			List<LineSegment2D> lineSegmentsInPlane = PolygonizerTools.toLineSegmentsInPlane(lineSegmentsToTransform, planeOrigin, planeNormal);
			
			assert(lineSegmentsInPlane.size() == 4);
			
			//System.out.println("testToLineSegmentsInPlaneVector3D: lineSegmentsInPlane = " + lineSegmentsInPlane.size());
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentsInPlaneVector3D: exception = " + e);
		}
	}

	@Test(timeout = 30000)
	public void testToLineSegmentsInPlaneOrientation()
	{
		initializeBasics();
		List<? extends LineSegment3DReadOnly> lineSegmentsToTransform = getLineConstraints3D();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();

		try
		{
			List<LineSegment2D> lineSegmentsInPlane = PolygonizerTools.toLineSegmentsInPlane(lineSegmentsToTransform, planeOrigin, planeOrientation);
			assert(lineSegmentsInPlane.size() == 4);

			//System.out.println("testToLineSegmentsInPlaneOrientation: lineSegmentsInPlane = " + lineSegmentsInPlane.size());
			for (LineSegment2D lineSegment : lineSegmentsInPlane)
			{
//				assertEquals(lineSegment.getFirstEndpointX(), 0.0, EPS);
//				assertEquals(lineSegment.getFirstEndpointY(), 0.0, EPS);
//				assertEquals(lineSegment.getSecondEndpointX(), 0.0, EPS);
//				assertEquals(lineSegment.getSecondEndpointY(), 1.0, EPS);
				//System.out.println("testToLineSegmentsInPlaneOrientation: lineSegment = " + lineSegment.getFirstEndpointX() + " " + lineSegment.getFirstEndpointY()
				//      + " " + lineSegment.getSecondEndpointX() + " " + lineSegment.getSecondEndpointY());
			}
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentsInPlaneOrientation: exception = " + e);
		}
	}

	@Test(timeout = 30000)
	public void testToLineSegmentInPlaneVector3D()
	{
		initializeBasics();
		LineSegment3DReadOnly lineSegmentToTransform = new LineSegment3D(0, 0, 0, 1, 1, 1);
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Vector3DReadOnly planeNormal = new Vector3D(0, 1, 0);

		try
		{
			LineSegment2D lineSegmentInPlane = PolygonizerTools.toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, planeNormal);

			assertEquals(lineSegmentInPlane.getFirstEndpointX(), 0.0, EPS);
			assertEquals(lineSegmentInPlane.getFirstEndpointY(), 0.0, EPS);
			assertEquals(lineSegmentInPlane.getSecondEndpointX(), 1.0, EPS);
			assertEquals(lineSegmentInPlane.getSecondEndpointY(), -1.0, EPS);
			//System.out.println("testToLineSegmentInPlaneVector3D: lineSegmentInPlane = " + lineSegmentInPlane);
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentInPlaneVector3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToLineSegmentInPlaneOreintation3D()
	{
		initializeBasics();
		LineSegment3DReadOnly lineSegmentToTransform = new LineSegment3D(0, 0, 0, 1, 1, 1);
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);

		try
		{
			LineSegment2D lineSegmentInPlane = PolygonizerTools.toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, planeOrientation);

			assertEquals(lineSegmentInPlane.getFirstEndpointX(), 0.0, EPS);
			assertEquals(lineSegmentInPlane.getFirstEndpointY(), 0.0, EPS);
			assertEquals(lineSegmentInPlane.getSecondEndpointX(), -1.0, EPS);
			assertEquals(lineSegmentInPlane.getSecondEndpointY(), 1.0, EPS);
			//System.out.println("testToLineSegmentInPlaneOreintation3D: lineSegmentInPlane = " + lineSegmentInPlane);
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentInPlaneOreintation3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToPointInPlane()
	{
		initializeBasics();
		double xToTransform = 1;
		double yToTransform = 1;
		double zToTransform = 1;
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);

		try
		{
			Point2D pointInPlane = PolygonizerTools.toPointInPlane(xToTransform, yToTransform, zToTransform, planeOrigin, planeOrientation);

			assertEquals(pointInPlane.getX(), -1, EPS);
			assertEquals(pointInPlane.getY(), 1, EPS);
			//System.out.println("testToPointInPlane: pointInPlane = " + pointInPlane.getX() + " " + pointInPlane.getY());
		}
		catch (Exception e)
		{
			System.out.println("testToPointInPlane: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToPointsInWorldVector3D()
	{
		initializeBasics();
		List<? extends Point2DReadOnly> pointsInPlane = getPointcloud2D();
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Vector3DReadOnly planeNormal = new Vector3D(0, 1, 0);

		try
		{
			List<Point3D> pointsInWorld = PolygonizerTools.toPointsInWorld(pointsInPlane, planeOrigin, planeNormal);
			
			assert(pointsInWorld.size() == 74);
			//System.out.println("testToPointsInWorldVector3D: pointsInWorld = " + pointsInWorld.size());
		}
		catch (Exception e)
		{
			System.out.println("testToPointsInWorldVector3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToPointsInWorldOrientation3D()
	{
		initializeBasics();
		List<? extends Point2DReadOnly> pointsInPlane = getPointcloud2D();
		Point3DReadOnly planeOrigin = new Point3D();
		Orientation3DReadOnly planeOrientation = new AxisAngle();

		try
		{
			List<Point3D> pointsInWorld = PolygonizerTools.toPointsInWorld(pointsInPlane, planeOrigin, planeOrientation);
			
			assert(pointsInWorld.size() == 74);
			//System.out.println("testToPointsInWorldOrientation3D: pointsInWorld = " + pointsInWorld.size());
		}
		catch (Exception e)
		{
			System.out.println("testToPointsInWorldOrientation3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToPointInWorldOrientation3D()
	{
		initializeBasics();
		Point2DReadOnly point2dReadOnly = new Point2D(1, 1);
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);
		
		try
		{

			Point3D pointInWorld = PolygonizerTools.toPointInWorld(point2dReadOnly, planeOrigin, planeOrientation);

			assertEquals(pointInWorld.getX(), 0.0, EPS);
			assertEquals(pointInWorld.getY(), 1.0, EPS);
			assertEquals(pointInWorld.getZ(), -1.0, EPS);
			//System.out.println("testToPointInWorldOrientation3D: pointInWorld = " + pointInWorld.getX() + " " + pointInWorld.getY() + " " + pointInWorld.getZ());
		}
		catch (Exception e)
		{
			System.out.println("testToPointInWorldOrientation3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToLineSegmentsInWorldVector3D()
	{
		initializeBasics();
		List<? extends LineSegment2DReadOnly> lineSegmentsToTransform = getLineConstraints2D();
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		try
		{
			List<LineSegment3D> lineSegmentsInWorld = PolygonizerTools.toLineSegmentsInWorld(lineSegmentsToTransform, planeOrigin, planeNormal);
			
			assert(lineSegmentsInWorld.size() == 4);
			//System.out.println("testToLineSegmentsInWorldVector3D: lineSegmentsInWorld = " + lineSegmentsInWorld.size());
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentsInWorldVector3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testtoLineSegmentsInWorldOrientation3D()
	{
		initializeBasics();
		List<? extends LineSegment2DReadOnly> lineSegmentsToTransform = getLineConstraints2D();
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI/2, 0);

		try
		{
			List<LineSegment3D> lineSegmentsInWorld = PolygonizerTools.toLineSegmentsInWorld(lineSegmentsToTransform, planeOrigin, planeOrientation);
			
			assert(lineSegmentsInWorld.size() == 4);
			//System.out.println("testToLineSegmentsInWorldOrientation3D: lineSegmentsInWorld  = " + lineSegmentsInWorld.size());
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentsInWorldOrientation3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToLineSegmentInWorldVector3D()
	{
		initializeBasics();
		LineSegment2DReadOnly lineSegmentToTransform = new LineSegment2D(0, 0, 1, 1);
		Point3DReadOnly planeOrigin = new Point3D();
		Vector3DReadOnly planeNormal = new Vector3D();

		try
		{
			LineSegment3D lineSegmentInWorld = PolygonizerTools.toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, planeNormal);

			assertEquals(lineSegmentInWorld.getFirstEndpointX(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getFirstEndpointY(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getFirstEndpointZ(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getSecondEndpointX(), 1.0, EPS);
			assertEquals(lineSegmentInWorld.getSecondEndpointY(), -1.0, EPS);
			assertEquals(lineSegmentInWorld.getSecondEndpointZ(), 0.0, EPS);
			//System.out.println("testToLineSegmentInWorldVector3D: lineSegmentsInWorld  = " + lineSegmentInWorld);
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentInWorldVector3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToLineSegmentInWorldOrientation3D()
	{
		initializeBasics();
		LineSegment2DReadOnly lineSegmentToTransform = new LineSegment2D(0, 0, 1, 1);
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);

		try
		{
			LineSegment3D lineSegmentInWorld = PolygonizerTools.toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, planeOrientation);

			assertEquals(lineSegmentInWorld.getFirstEndpointX(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getFirstEndpointY(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getFirstEndpointZ(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getSecondEndpointX(), 0.0, EPS);
			assertEquals(lineSegmentInWorld.getSecondEndpointY(), 1.0, EPS);
			assertEquals(lineSegmentInWorld.getSecondEndpointZ(), -1.0, EPS);
			//System.out.println("testToLineSegmentInWorldOrientation3D: lineSegmentInWorld  = " + lineSegmentInWorld);
		}
		catch (Exception e)
		{
			System.out.println("testToLineSegmentInWorldVector3D: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testToPointInWorld()
	{
		initializeBasics();
		double xToTransform = 1;
		double yToTransform = 1;
		Point3DReadOnly planeOrigin = new Point3D(0, 0, 0);
		Orientation3DReadOnly planeOrientation = new AxisAngle(0, Math.PI / 2, 0);

		try
		{
			Point3D pointInWorld = PolygonizerTools.toPointInWorld(xToTransform, yToTransform, planeOrigin, planeOrientation);

			assertEquals(pointInWorld.getX(), 0, EPS); //new Point3D( 0, 1, -1 ));
			assertEquals(pointInWorld.getY(), 1, EPS); //new Point3D( 0, 1, -1 ));
			assertEquals(pointInWorld.getZ(), -1, EPS); //new Point3D( 0, 1, -1 ));
			//System.out.println("testToPointInWorld: pointInWorld = " + pointInWorld.getX() + " " + pointInWorld.getY() + " " + pointInWorld.getZ());
		}
		catch (Exception e)
		{
			System.out.println("testToPointInWorld: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testGetQuaternionFromZUpToVector()
	{
		initializeBasics();
		Vector3DReadOnly normal = new Vector3D(0, 1, 0);

		try
		{
			Quaternion quaternionFromZUpToVector = PolygonizerTools.getQuaternionFromZUpToVector(normal);

			assertEquals(quaternionFromZUpToVector.getX(), -0.70710, EPS);
			assertEquals(quaternionFromZUpToVector.getY(), 0, EPS);
			assertEquals(quaternionFromZUpToVector.getZ(), 0, EPS);
			assertEquals(quaternionFromZUpToVector.getS(), 0.70710, EPS);
			//System.out.println("testGetQuaternionFromZUpToVector: quaternionFromZUpToVector  = " + quaternionFromZUpToVector.getX() + " " + quaternionFromZUpToVector.getY() + " " + quaternionFromZUpToVector.getZ() + " " + quaternionFromZUpToVector.getS());
		}
		catch (Exception e)
		{
			System.out.println("testGetQuaternionFromZUpToVector: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testComputeEllipsoidVolumeRadii()
	{
		initializeBasics();
		Vector3DReadOnly radii = new Vector3D(1, 1, 1);

		try
		{
			double ellipsoidVolume = PolygonizerTools.computeEllipsoidVolume(radii);

			assertEquals(ellipsoidVolume, 4.18879, EPS);
			//System.out.println("testComputeEllipsoidVolumeRadii: ellipsoidVolume = " + ellipsoidVolume);
		}
		catch (Exception e)
		{
			System.out.println("testComputeEllipsoidVolumeRadii: exception = " + e);
		}

	}

	@Test(timeout = 30000)
	public void testComputeEllipsoidVolumeXYZ()
	{
		initializeBasics();
		double xRadius = 1;
		double yRadius = 1;
		double zRadius = 1;

		try
		{
			double ellipsoidVolume = PolygonizerTools.computeEllipsoidVolume(xRadius, yRadius, zRadius);

			assertEquals(ellipsoidVolume, 4.18879, EPS);
			//System.out.println("testComputeEllipsoidVolumeXYZ: ellipsoidVolume = " + ellipsoidVolume);
		}
		catch (Exception e)
		{
			System.out.println("testComputeEllipsoidVolumeXYZ: exception = " + e);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
	public final void testPlanarRegionPolygonizer()
	{
		initializeBasics();
		List<Point3D> pointcloud = new ArrayList<>();
		pointcloud.add(new Point3D(0.5, -0.1, 0.0));
		pointcloud.add(new Point3D(0.5, 0.1, 0.0));
		pointcloud.add(new Point3D(1.5, 0.1, 0.0));
		pointcloud.add(new Point3D(1.5, -0.1, 0.0));

		// pointcloud.add(new Point3D(0.5, -0.1, 0.0));
		pointcloud.add(new Point3D(1.4, 1.0, 0.0));
		pointcloud.add(new Point3D(1.5, 1.0, 0.0));
		pointcloud.add(new Point3D(1.4, 0.10, 0.0));
		//System.out.println("\npointcloud: " + pointcloud.toString());

		List<LineSegment3D> lineConstraints = new ArrayList<>();
		lineConstraints.add(new LineSegment3D(0.0, -0.5, 0.0, 0.0, 0.5, 0.0));
		lineConstraints.add(new LineSegment3D(2.0, -0.5, 0.0, 2.0, 0.5, 0.0));
		lineConstraints.add(new LineSegment3D(0.0, 0.5, 0.0, 2.0, 0.5, 0.0));
		lineConstraints.add(new LineSegment3D(0.0, -0.5, 0.0, 2.0, -0.5, 0.0));
		//System.out.println("\nlineConstraints: " + lineConstraints.toString());

		// PlanarRegionSegmentationRawData
		rawData = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud);
		rawData.addIntersections(lineConstraints);
		//System.out.println("PlanarRegionPolygonizerTest: rawData: " + rawData.toString());

		rawDataList.add(rawData);
		//System.out.println("PlanarRegionPolygonizerTest: rawDataList: " + rawDataList.toString());

		concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
		concaveHullFactoryParameters.setTriangulationTolerance(1.0e-5);
		concaveHullFactoryParameters.setEdgeLengthThreshold(0.15);
		//System.out.println("\nparameters: " + concaveHullFactoryParameters.toString());

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
		//System.out.println("PlanarRegionPolygonizerTest: concaveHullCollection: " + concaveHullCollection.getConcaveHulls().toString());

		assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());

		Object[] hull = concaveHullCollection.getConcaveHulls().toArray();
		//for (int i = 0; i < hull.length; i++)
		//	System.out.println("PlanarRegionPolygonizerTest: hull[" + i + "]: " + hull[i].toString());

		//ConcaveHullPocket  chp =  findFirstConcaveHullPocket(concaveHullVertices);

	}

}
