package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import javafx.geometry.Point3D;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotEnvironmentAwareness.geometry.REAGeometryTools;

public class REAGeometryToolsTest
{
	private static final double EPS = 10;
	Random random = new Random();
	private final BoundingBox3D boundingBox1 = new BoundingBox3D();
	private final BoundingBox3D boundingBox2 = new BoundingBox3D();

	REAGeometryTools rt = new REAGeometryTools();

	@SuppressWarnings("deprecation")
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
	public final void testREAGeometryTools()
	{
		double[] newMin1 = {0, 0, 0}, newMax1 = {1, 1, 1}, newMin2 = {1, 1, 1}, newMax2 = {2, 2, 2};
		boundingBox1.set(newMin1, newMax1);
		boundingBox1.set(newMin2, newMax2);

		Point3DReadOnly min1 = boundingBox1.getMinPoint();
		Point3DReadOnly max1 = boundingBox1.getMaxPoint();
		Point3DReadOnly min2 = boundingBox2.getMinPoint();
		Point3DReadOnly max2 = boundingBox2.getMaxPoint();

		double distanceSquared = REAGeometryTools.distanceSquaredBetweenTwoBoundingBox3Ds(min1, max1, min2, max2);
		assertNotEquals(distanceSquared, 3);
	}
}
