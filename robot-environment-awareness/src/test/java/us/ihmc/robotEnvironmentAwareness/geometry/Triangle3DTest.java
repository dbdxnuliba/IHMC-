package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Tuple3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class Triangle3DTest
{
	private static final int NUMBER_OF_ITERATIONS = 10000;
	private static final double EPS = 1.0e-15;
	Random random = new Random();

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test

	// Tests Triangle3D creation, setters(), applyTransform(),
	// applyInverseTransform() and geometricallyEqual()

	public final void testTriangle3D()
	{
		for (int iter = 0; iter < NUMBER_OF_ITERATIONS; iter++)
		{
			Triangle3D randomTriangle = new Triangle3D();
			Triangle3D referenceTriangle = new Triangle3D();
			Vector3D[] randomVector = new Vector3D[3];
			RigidBodyTransform transform = new RigidBodyTransform();

			randomTriangle.setToNaN();
			assertTrue(randomTriangle.containsNaN());
			assertFalse(randomTriangle.geometricallyEquals(referenceTriangle, EPS));

			randomTriangle.setToZero();
			assertTrue(randomTriangle.geometricallyEquals(referenceTriangle, EPS));

			for (int i = 0; i < 3; i++)
				randomVector[i] = RandomGeometry.nextVector3D(random);

			randomTriangle.set(randomVector[0], randomVector[1], randomVector[2]);
			referenceTriangle.set(randomTriangle);

			// Now construct a random transform to manipulate the random triangle...
			double roll = (random.nextDouble() - 0.5) * Math.PI * 2.0;
			double pitch = (random.nextDouble() - 0.5) * Math.PI * 2.0;
			double yaw = (random.nextDouble() - 0.5) * Math.PI * 2.0;
			double x = random.nextDouble() - 0.5;
			double y = random.nextDouble() - 0.5;
			double z = random.nextDouble() - 0.5;

			transform.appendRollRotation(roll);
			transform.appendPitchRotation(pitch);
			transform.appendYawRotation(yaw);
			transform.appendTranslation(x, y, z);

			randomTriangle.applyTransform(transform);
			randomTriangle.applyInverseTransform(transform);

			assertTrue(randomTriangle.geometricallyEquals(referenceTriangle, EPS));
			assertFalse(randomTriangle.geometricallyEquals(new Triangle3D(), EPS));

			referenceTriangle.set(randomTriangle.getB(), randomTriangle.getC(), randomTriangle.getA());
			assertTrue(randomTriangle.geometricallyEquals(referenceTriangle, EPS));

			referenceTriangle.set(randomTriangle.getC(), randomTriangle.getA(), randomTriangle.getB());
			assertTrue(randomTriangle.geometricallyEquals(referenceTriangle, EPS));

		}
	}
}
