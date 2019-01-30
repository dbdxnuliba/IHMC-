package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;

public class PointMeanTest
{
	private static final int NUMBER_OF_ITERATIONS = 10000;
	private static final double EPS = 1.0e-12;
	Random random = new Random();

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test
	public final void testVectorMean()
	{
		Point3D p = new Point3D();
		Point3D sum = new Point3D();
		PointMean pMean = new PointMean();

		for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
		{
			p = RandomGeometry.nextPoint3D(random, 0, 1);
			pMean.update(p);
			sum.add(p);
		}

		sum.scale(1.0 / NUMBER_OF_ITERATIONS);

		assertEquals(pMean.getX(), sum.getX(), EPS);
		assertEquals(pMean.getY(), sum.getY(), EPS);
		assertEquals(pMean.getZ(), sum.getZ(), EPS);

		pMean.clear();
		assertEquals(pMean.getX(), 0, EPS);
		assertEquals(pMean.getY(), 0, EPS);
		assertEquals(pMean.getZ(), 0, EPS);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test
	public final void testWithTuple3DBasics()
	{
		Point3D p = new Point3D();
		Point3D sum = new Point3D();
		PointMean pMean = new PointMean();

		for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
		{
			p = RandomGeometry.nextPoint3D(random, 0, 1);
			pMean.update((Tuple3DBasics) p, 1);
			sum.add(p);
		}

		sum.scale(1.0 / NUMBER_OF_ITERATIONS);

		assertEquals(pMean.getX(), sum.getX(), EPS);
		assertEquals(pMean.getY(), sum.getY(), EPS);
		assertEquals(pMean.getZ(), sum.getZ(), EPS);
		assertEquals(pMean.getNumberOfSamples(), NUMBER_OF_ITERATIONS);

		pMean.clear();
		assertEquals(pMean.getX(), 0, EPS);
		assertEquals(pMean.getY(), 0, EPS);
		assertEquals(pMean.getZ(), 0, EPS);
	}

}
