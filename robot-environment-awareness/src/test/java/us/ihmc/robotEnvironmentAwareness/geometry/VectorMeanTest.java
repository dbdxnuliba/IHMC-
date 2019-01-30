package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;

public class VectorMeanTest
{
	private static final int NUMBER_OF_ITERATIONS = 10000;
	private static final double EPS = 1.0e-12;
	Random random = new Random();

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test
	public final void testWithVectorMean()
	{
		VectorMean meanVector = new VectorMean();
		Vector3D randomVector = new Vector3D();
		Vector3D sum = new Vector3D();

		meanVector.clear();

		for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
		{
			randomVector = RandomGeometry.nextVector3D(random, 1);
			meanVector.update(randomVector);
			sum.add(randomVector);
		}

		sum.scale(1.0 / NUMBER_OF_ITERATIONS);

		assertEquals(meanVector.getX(), sum.getX(), EPS);
		assertEquals(meanVector.getY(), sum.getY(), EPS);
		assertEquals(meanVector.getZ(), sum.getZ(), EPS);
		assertEquals(meanVector.getNumberOfSamples(), NUMBER_OF_ITERATIONS);

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test
	public final void testWithTuple3DBasics()
	{
		VectorMean meanVector = new VectorMean();
		Vector3D randomVector = new Vector3D();
		Vector3D sum = new Vector3D();

		for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
		{
			randomVector = RandomGeometry.nextVector3D(random, 1);
			meanVector.update((Tuple3DBasics) randomVector, 1);
			sum.add(randomVector);
		}

		sum.scale(1.0 / NUMBER_OF_ITERATIONS);

		assertEquals(meanVector.getX(), sum.getX(), EPS);
		assertEquals(meanVector.getY(), sum.getY(), EPS);
		assertEquals(meanVector.getZ(), sum.getZ(), EPS);
		assertEquals(meanVector.getNumberOfSamples(), NUMBER_OF_ITERATIONS);

		meanVector.clear();
		assertEquals(meanVector.getX(), 0, EPS);
		assertEquals(meanVector.getY(), 0, EPS);
		assertEquals(meanVector.getZ(), 0, EPS);
	}

}
